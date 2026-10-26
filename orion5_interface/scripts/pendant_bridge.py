#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pendant_bridge.py — Mery6 web pendant <-> ROS / MoveIt bridge (SIMULATION ONLY).

Architecture:
    Web pendant (roslibjs)  --/pendant/cmd (std_msgs/String, JSON)-->  this node
    this node               --/pendant/status (JSON, 10 Hz)--------->  Web pendant
    this node               --/pendant/robot_info (JSON, latched)--->  Web pendant

    this node -> MoveIt (moveit_commander: IK, joint-limit + collision checked planning)
              -> FollowJointTrajectory action on /orion_arm_pos_joint_traj_controller
              -> Gazebo (ros_control, PositionJointInterface)
              -> /mcfly_grippers/mc1_gripper/angle service for the MC1 gripper

The pendant itself keeps the manual joint jog (direct JointTrajectory on the controller
/command topic) and /joint_states monitoring, so jog and monitoring work even if this
node is down.  Everything that needs planning, completion checks, cancellation, taught
poses or task sequencing lives here.

States: IDLE, PLANNING, EXECUTING, TARGET_REACHED, FAILED, STOPPED, ESTOP

Limitations (simulation):
  * The E-STOP handled here is a *simulation* E-STOP (cancels goals, holds controllers,
    rejects new commands). It is NOT a safety-rated industrial emergency stop.
  * There is no grasp/attach plugin: gripper "CLOSED" means the fingers reached the
    closed angle; an object is never attached to the gripper.
  * The MC1 links have no collision geometry; an approximate box attached to eef_link in
    the planning scene stands in for the gripper (planning only, not Gazebo physics).
"""

import collections
import csv
import json
import math
import os
import queue
import re
import threading
import time
import traceback

import rospy
import actionlib
import moveit_commander
import yaml

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from mcfly_grippers.srv import GripperControl
from gazebo_msgs.msg import ModelStates, LinkStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
try:
    from gazebo_ros_link_attacher.srv import Attach, AttachRequest
    HAVE_ATTACHER = True
except ImportError:      # package not built in the workspace: attach/detach steps fail with a clear message
    Attach = AttachRequest = None
    HAVE_ATTACHER = False
import tf.transformations as tft

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIP_JOINTS = ["grip_joint_1_left", "grip_joint_2_left", "grip_joint_3_left",
               "grip_joint_1_right", "grip_joint_2_right", "grip_joint_3_right"]
GRIP_LINKS = ["gripper_base", "left_link_1", "left_link_2", "left_link_3", "left_finger",
              "right_link_1", "right_link_2", "right_link_3", "right_finger", "gripper_tcp_link"]

ARM_CONTROLLER = "orion_arm_pos_joint_traj_controller"
GRIPPER_CONTROLLER = "mc1_traj_controller"
GRIPPER_SERVICE = "/mcfly_grippers/mc1_gripper/angle"
PLANNING_GROUP = "orion5_arm"
EEF_LINK = "eef_tcp_link"          # MoveIt tip (= MC1 fingertip TCP)
EEF_PARENT_LINK = "eef_link"       # flange link the gripper (and the envelope box) hang from
REF_FRAME = "base_link"
ENVELOPE_NAME = "gripper_envelope"
ATTACH_SERVICE = "/link_attacher_node/attach"
DETACH_SERVICE = "/link_attacher_node/detach"
SET_MODEL_STATE_SERVICE = "/gazebo/set_model_state"
HISTORY_COLS = ["time", "scenario", "task", "result", "cycle_time_s", "cycle_time_sim_s", "speed_pct", "steps_total",
                "steps_done", "failure_step", "message", "object", "object_at_place", "object_attached", "rtf", "steps"]

STATE_IDLE = "IDLE"
STATE_PLANNING = "PLANNING"
STATE_EXECUTING = "EXECUTING"
STATE_REACHED = "TARGET_REACHED"
STATE_FAILED = "FAILED"
STATE_STOPPED = "STOPPED"
STATE_ESTOP = "ESTOP"

GRASP_NOTE = "fingers at closed angle - the object is carried by a kinematic link attachment (simulation), not by a physics grasp"
TAUGHT_NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,31}$")

GOAL_STATUS_NAMES = {v: k for k, v in GoalStatus.__dict__.items()
                     if isinstance(v, int) and k.isupper()}
MOVEIT_ERROR_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items()
                      if isinstance(v, int) and k.isupper()}


def deg2rad(v):
    return [math.radians(x) for x in v]


def rad2deg(v):
    return [math.degrees(x) for x in v]


class StopRequested(Exception):
    pass


class JointStateCache(object):
    """Name-indexed cache of the latest /joint_states values."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pos = {}
        self.vel = {}
        self.eff = {}
        self.stamp = 0.0
        self.wall_stamp = 0.0     # wall-clock receipt time (freshness check independent of sim time)

    def callback(self, msg):
        with self.lock:
            self.wall_stamp = time.time()
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.pos[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.vel[name] = msg.velocity[i]
                if i < len(msg.effort):
                    self.eff[name] = msg.effort[i]
            self.stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

    def positions(self, names):
        with self.lock:
            if not all(n in self.pos for n in names):
                return None
            return [self.pos[n] for n in names]

    def velocities(self, names):
        with self.lock:
            return [self.vel.get(n, 0.0) for n in names]

    def efforts(self, names):
        with self.lock:
            return [self.eff.get(n, 0.0) for n in names]


def load_urdf_limits():
    """Joint limits from /robot_description (rad, rad/s)."""
    limits = {}
    try:
        from urdf_parser_py.urdf import URDF
        robot = URDF.from_parameter_server()
        for j in robot.joints:
            if j.type in ("revolute", "prismatic") and j.limit is not None:
                limits[j.name] = {"lower": float(j.limit.lower), "upper": float(j.limit.upper),
                                  "velocity": float(j.limit.velocity)}
    except Exception as exc:  # pragma: no cover
        rospy.logwarn("pendant_bridge: could not parse URDF limits: %s", exc)
    return limits


class PendantBridge(object):

    def __init__(self):
        # ---- parameters -------------------------------------------------
        self.status_rate = rospy.get_param("~status_rate", 10.0)
        self.joint_tol = rospy.get_param("~joint_tol_rad", 0.02)
        self.vel_tol = rospy.get_param("~vel_tol_rad_s", 0.02)
        self.settle_time = rospy.get_param("~settle_time", 0.2)
        self.speed_pct = float(rospy.get_param("~default_speed_pct", 30))
        self.gripper_open_deg = float(rospy.get_param("~gripper_open_deg", 0.0))
        self.gripper_close_deg = float(rospy.get_param("~gripper_close_deg", 35.0))
        self.gripper_time = float(rospy.get_param("~gripper_time", 1.0))
        self.gripper_tol_deg = float(rospy.get_param("~gripper_tol_deg", 2.0))
        self.cart_eef_step = float(rospy.get_param("~cartesian_eef_step", 0.005))
        self.cart_min_fraction = float(rospy.get_param("~cartesian_min_fraction", 0.95))
        self.cart_max_jump = float(rospy.get_param("~cartesian_max_joint_jump_rad", 0.35))
        # Named joint poses (deg). Only HOME is shipped by default (all zero = SRDF 'home').
        self.named_poses = {str(k).upper(): v for k, v in
                            rospy.get_param("~named_poses", {"HOME": [0, 0, 0, 0, 0, 0]}).items()}
        self.tasks = rospy.get_param("~tasks", {}) or {}
        self.workcell = rospy.get_param("~workcell", []) or []
        self.envelope = rospy.get_param("~gripper_envelope",
                                        {"enabled": True, "size": [0.07, 0.07, 0.17], "z_offset": 0.085}) or {}
        default_taught = ""
        try:
            import rospkg
            default_taught = os.path.join(rospkg.RosPack().get_path("orion5_interface"), "config", "scenarios",
                                          "%s_taught_poses.yaml" % rospy.get_param("~scenario/name", "barista"))
        except Exception:
            pass
        self.taught_file = rospy.get_param("~taught_poses_file", default_taught)
        # Gazebo objects handled by task steps (simulation-only kinematic attachment, see config/pendant_bridge.yaml)
        self.robot_model = str(rospy.get_param("~robot_model", "orion5_v166"))
        self.gazebo_z_offset = float(rospy.get_param("~gazebo_z_offset", 0.40))
        self.gripper_link_candidates = list(rospy.get_param("~gripper_link_candidates", ["gripper_base", "eef_link", "link_6"]))
        # Execution clock: controllers run on simulation time (/clock). All execution / arrival / gripper timeouts are
        # measured in that clock so a slow Gazebo (real-time factor < 1, typical in a VM) does not cause false timeouts.
        # A wall-clock cap (wall_cap_factor x sim timeout, at least 30 s) still ends the wait if the simulation is paused.
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))
        self.wall_cap_factor = float(rospy.get_param("~wall_cap_factor", 4.0))
        self.rtf = None                      # measured real-time factor (sim seconds per wall second), None until sampled
        self._rtf_sample = None
        # Task history: every finished cycle is appended to a CSV (operator record / measurement source) and the last
        # entries are published in the status (task_history).
        self.history_file = rospy.get_param("~task_history_file", "")
        self.history = collections.deque(maxlen=20)
        self.objects = {str(o["name"]): o for o in (rospy.get_param("~objects", []) or []) if o.get("name")}
        # Scenario metadata (config/scenarios/<name>.yaml): name, description, world. Shown on the pendant.
        self.scenario = rospy.get_param("~scenario", {}) or {}
        if not isinstance(self.scenario, dict):
            self.scenario = {"name": str(self.scenario)}

        self.limits = load_urdf_limits()

        # ---- state --------------------------------------------------------
        self.lock = threading.Lock()
        self.state = STATE_IDLE
        self.estop = False
        self.stop_evt = threading.Event()
        self.last_error = ""
        self.last_cmd = {"id": None, "cmd": None, "result": None, "message": ""}
        self.target_rad = None              # dict joint -> rad (last commanded arm target)
        # plan/exec are reset per queued command (tagged with its cmd_id); "attempted" tells whether the
        # stage actually started, so a pre-planning rejection leaves attempted == False.
        self.plan_info = self._empty_plan(None)
        self.exec_info = self._empty_exec(None)
        self.busy = False                   # worker is processing a command (planning or executing)
        self.step_outcome = None            # "skipped" for conditional task steps that did nothing
        self.step_note = None               # human-readable note for the current task step
        self.collision = {"valid": None, "contacts": []}
        self.gripper = {"cmd_deg": None, "angle_deg": None, "state": "UNKNOWN", "reached": None,
                        "note": "CLOSED = fingers at closed angle; the object is carried by a kinematic attachment (simulation)"}
        self.task = {"name": None, "running": False, "step_index": -1, "step_count": 0,
                     "step_name": None, "steps": [], "cycle_time_s": 0.0,
                     "last_cycle_time_s": None, "last_cycle_time_sim_s": None, "count": 0, "result": None, "message": ""}
        self.teach_last = None
        self.task_t0 = None
        self.workcell_loaded = []
        self.taught = {}
        self.model_poses = {}        # Gazebo model name -> (x, y, z) in the Gazebo world frame
        self.model_poses_wall = 0.0
        self.gazebo_links = set()    # "model::link" names seen once on /gazebo/link_states
        self.attached = {}           # object name -> gripper link it is attached to (bridge-side bookkeeping)
        self.set_state_srv = None
        self.attach_srv = None
        self.detach_srv = None

        # ---- ROS interfaces ----------------------------------------------
        self.js = JointStateCache()
        rospy.Subscriber("/joint_states", JointState, self.js.callback, queue_size=1)

        self.arm_cmd_pub = rospy.Publisher("/%s/command" % ARM_CONTROLLER, JointTrajectory, queue_size=1)
        self.grip_cmd_pub = rospy.Publisher("/%s/command" % GRIPPER_CONTROLLER, JointTrajectory, queue_size=1)
        self.status_pub = rospy.Publisher("/pendant/status", String, queue_size=1)
        self.info_pub = rospy.Publisher("/pendant/robot_info", String, queue_size=1, latch=True)

        rospy.loginfo("pendant_bridge: waiting for MoveIt group '%s' ...", PLANNING_GROUP)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)
        self.group.set_end_effector_link(EEF_LINK)
        self.group.set_pose_reference_frame(REF_FRAME)
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(3)
        self.group.set_goal_joint_tolerance(0.001)
        self.planning_frame = self.group.get_planning_frame()
        if self.planning_frame.strip("/") not in ("world", "base_link"):
            rospy.logwarn("pendant_bridge: planning frame is '%s' (expected world/base_link, identity to base_link)",
                          self.planning_frame)
        self.srdf_names = list(self.group.get_named_targets())

        self.fjt = actionlib.SimpleActionClient("/%s/follow_joint_trajectory" % ARM_CONTROLLER,
                                                FollowJointTrajectoryAction)
        if not self.fjt.wait_for_server(rospy.Duration(15.0)):
            rospy.logwarn("pendant_bridge: FollowJointTrajectory server not available yet")

        self.gripper_srv = None
        try:
            rospy.wait_for_service(GRIPPER_SERVICE, timeout=10.0)
            self.gripper_srv = rospy.ServiceProxy(GRIPPER_SERVICE, GripperControl)
        except rospy.ROSException:
            rospy.logwarn("pendant_bridge: gripper service %s not available", GRIPPER_SERVICE)

        self.validity_srv = None
        try:
            rospy.wait_for_service("/check_state_validity", timeout=10.0)
            self.validity_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        except rospy.ROSException:
            rospy.logwarn("pendant_bridge: /check_state_validity not available")

        self.taught = self._load_taught()
        self.add_workcell_objects()
        self._init_gazebo_objects()

        # ---- worker + timers ---------------------------------------------
        self.jobs = queue.Queue()
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

        rospy.Subscriber("/pendant/cmd", String, self.on_cmd, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / self.status_rate), self.publish_status)
        rospy.Timer(rospy.Duration(0.5), self.check_collision)
        threading.Thread(target=self._rtf_loop, daemon=True).start()   # wall-clock thread: keeps sampling even if sim pauses

        self.publish_info()
        rospy.loginfo("pendant_bridge: ready (planning frame %s, eef %s, %d taught poses, workcell %s)",
                      self.planning_frame, EEF_LINK, len(self.taught), self.workcell_loaded)

    # ------------------------------------------------------------------ info
    @staticmethod
    def _empty_plan(cmd_id):
        return {"cmd_id": cmd_id, "attempted": False, "ok": None, "planner": None, "time_s": None,
                "traj_duration_s": None, "points": None, "fraction": None, "error_code": None}

    @staticmethod
    def _empty_exec(cmd_id):
        return {"cmd_id": cmd_id, "attempted": False, "goal_status": None, "elapsed_s": None, "timeout_s": None}

    def _resolve_named(self, name):
        """Single pose-resolution path used by move_named, task steps, requirements and resolve_pose.
        Order: named_poses (yaml) -> taught poses -> SRDF group states.
        Returns (source, joints_deg | None, srdf_name | None); source None = unknown."""
        key = str(name).upper()
        if key in self.named_poses:
            return "named_poses", list(self.named_poses[key]), None
        if key in self.taught:
            return "taught", list(self.taught[key]["joints_deg"]), None
        if name in self.srdf_names:
            return "srdf", None, name
        return None, None, None

    def _resolved_joints_deg(self, name):
        """Joint values (deg) the bridge would actually command for `name`, or None if unknown."""
        source, joints_deg, srdf = self._resolve_named(name)
        if source is None:
            return None
        if joints_deg is None and srdf is not None:
            vals = self.group.get_named_target_values(srdf)
            joints_deg = rad2deg([vals[n] for n in ARM_JOINTS])
        return joints_deg

    def task_requirements(self, task_name):
        """Pose names a task needs and which of them are not available yet."""
        task = self.tasks.get(task_name) or {}
        required, missing = [], []
        entries = []   # (POSE_KEY, raw_name, needs_taught_tcp)
        for s in task.get("steps", []):
            t = s.get("type")
            if t in ("named", "named_if_needed") and s.get("pose"):
                entries.append((str(s["pose"]).upper(), s["pose"], False))
            if t == "named_if_needed" and s.get("unless_reachable"):
                entries.append((str(s["unless_reachable"]).upper(), s["unless_reachable"], False))
            if t in ("cartesian_to", "verify_tcp") and s.get("pose"):
                entries.append((str(s["pose"]).upper(), s["pose"], True))      # needs a taught pose with a stored TCP
        for p, raw, needs_tcp in entries:
            if p not in required:
                required.append(p)
            ok = (p in self.taught) if needs_tcp else (self._resolve_named(raw)[0] is not None)
            if not ok and p not in missing:
                missing.append(p)
        return {"required": required, "missing": missing}

    def robot_info(self):
        joints = []
        for n in ARM_JOINTS:
            lim = self.limits.get(n, {"lower": -math.pi, "upper": math.pi, "velocity": 1.57})
            joints.append({"name": n,
                           "min_deg": round(math.degrees(lim["lower"]), 2),
                           "max_deg": round(math.degrees(lim["upper"]), 2),
                           "max_vel_dps": round(math.degrees(lim["velocity"]), 1)})
        tasks, reqs = {}, {}
        for name, t in self.tasks.items():
            tasks[name] = [s.get("name", s.get("type", "?")) for s in t.get("steps", [])]
            reqs[name] = self.task_requirements(name)
        return {"joints": joints,
                "gripper": {"joints": GRIP_JOINTS, "open_deg": self.gripper_open_deg,
                            "close_deg": self.gripper_close_deg},
                "named_poses": self.named_poses,
                "srdf_named_targets": self.srdf_names,
                "taught_poses": {k: {"joints_deg": v["joints_deg"], "tcp": v["tcp"]} for k, v in self.taught.items()},
                "taught_poses_file": self.taught_file,
                "tasks": tasks,
                "task_requirements": reqs,
                "workcell_loaded": list(self.workcell_loaded),
                "gripper_envelope": self.envelope,
                "objects": {n: {"gazebo_model": o.get("gazebo_model", n), "start_xyz_base": o.get("start_xyz_base"),
                                "place_xyz_base": o.get("place_xyz_base"), "tolerance_m": o.get("tolerance_m", 0.03)}
                            for n, o in self.objects.items()},
                "attacher_available": self.attach_srv is not None,
                "task_history_file": self.history_file,
                "scenario": {"name": self.scenario.get("name"), "description": self.scenario.get("description"),
                             "world": self.scenario.get("world")},
                "eef_link": EEF_LINK, "ref_frame": REF_FRAME,
                "estop_is_simulation_only": True,
                "limitations": ["simulation E-STOP only (not safety-rated)",
                                "object handling = kinematic link attachment (gazebo_ros_link_attacher), not a physics grasp; "
                                "a failed/stopped task never detaches automatically",
                                "gripper collision = approximate attached envelope box (planning only)",
                                "HOME is an elbow+wrist singularity: start Cartesian moves from READY"]}

    def publish_info(self):
        self.info_pub.publish(String(data=json.dumps(self.robot_info())))

    # ------------------------------------------------------------ workcell
    def add_workcell_objects(self):
        """(Re)insert static workcell objects (~workcell) and the gripper envelope into the planning scene."""
        expected = []
        rospy.sleep(0.5)  # let the scene interface connect
        for obj in self.workcell:
            name = obj.get("name")
            if not name:
                continue
            try:
                ps = PoseStamped()
                ps.header.frame_id = obj.get("frame", REF_FRAME)
                xyz = obj.get("xyz", [0, 0, 0])
                q = tft.quaternion_from_euler(*deg2rad(obj.get("rpy_deg", [0, 0, 0])))
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
                ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
                self.scene.remove_world_object(name)
                if obj.get("shape", "box") == "cylinder":
                    self.scene.add_cylinder(name, ps, height=obj["size"][1], radius=obj["size"][0])
                else:
                    self.scene.add_box(name, ps, size=tuple(obj["size"]))
                expected.append(name)
            except Exception as exc:
                rospy.logwarn("pendant_bridge: workcell object %s failed: %s", obj, exc)
        if self.envelope.get("enabled", True):
            try:
                self.scene.remove_attached_object(EEF_PARENT_LINK, name=ENVELOPE_NAME)
                self.scene.remove_world_object(ENVELOPE_NAME)
                ps = PoseStamped()
                ps.header.frame_id = EEF_PARENT_LINK
                ps.pose.position.z = float(self.envelope.get("z_offset", 0.085))
                ps.pose.orientation.w = 1.0
                touch = GRIP_LINKS + ["link_5", "link_6", EEF_PARENT_LINK, EEF_LINK]
                self.scene.attach_box(EEF_PARENT_LINK, ENVELOPE_NAME, pose=ps,
                                      size=tuple(self.envelope.get("size", [0.07, 0.07, 0.17])), touch_links=touch)
                expected.append(ENVELOPE_NAME)
            except Exception as exc:
                rospy.logwarn("pendant_bridge: gripper envelope failed: %s", exc)
        # verify
        t0 = time.time()
        present = set()
        while time.time() - t0 < 3.0 and not rospy.is_shutdown():
            try:
                present = set(self.scene.get_known_object_names()) | set(self.scene.get_attached_objects().keys())
            except Exception:
                present = set()
            if set(expected) <= present:
                break
            rospy.sleep(0.2)
        self.workcell_loaded = [n for n in expected if n in present]
        missing = [n for n in expected if n not in present]
        if missing:
            rospy.logwarn("pendant_bridge: planning-scene objects not confirmed: %s", missing)
        rospy.loginfo("pendant_bridge: planning scene objects: %s", self.workcell_loaded)

    # ------------------------------------------------------- taught poses
    def _load_taught(self):
        out = {}
        if not self.taught_file or not os.path.isfile(self.taught_file):
            return out
        try:
            with open(self.taught_file, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            rospy.logwarn("pendant_bridge: cannot read %s: %s", self.taught_file, exc)
            return out
        for name, rec in (data.get("taught_poses") or {}).items():
            try:
                name = str(name).upper()
                jd = [float(v) for v in rec["joints_deg"]]
                tcp = rec["tcp"]
                assert len(jd) == 6 and len(tcp["q"]) == 4
                err = self.validate_joint_limits(deg2rad(jd))
                if err:
                    rospy.logwarn("pendant_bridge: taught pose %s skipped: %s", name, err)
                    continue
                out[name] = {"joints_deg": jd, "tcp": tcp, "stamp": rec.get("stamp")}
            except Exception as exc:
                rospy.logwarn("pendant_bridge: taught pose %s skipped (%s)", name, exc)
        rospy.loginfo("pendant_bridge: %d taught poses loaded from %s", len(out), self.taught_file)
        return out

    def _save_taught(self):
        if not self.taught_file:
            raise RuntimeError("no taught_poses_file configured")
        d = os.path.dirname(self.taught_file)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = self.taught_file + ".tmp"
        with open(tmp, "w") as f:
            f.write("# Taught poses written by pendant_bridge.py (Teach button). joints_deg + TCP of eef_tcp_link in base_link.\n")
            yaml.safe_dump({"taught_poses": self.taught}, f, sort_keys=False, allow_unicode=True)
        os.replace(tmp, self.taught_file)

    def do_teach(self, cmd):
        name = str(cmd.get("name", "")).strip().upper()
        if not TAUGHT_NAME_RE.match(name):
            self.set_result(cmd, "error", "pose name must match ^[A-Z][A-Z0-9_]{0,31}$")
            return
        if name in self.named_poses or name.lower() in self.srdf_names:
            self.set_result(cmd, "error", "'%s' is a fixed named/SRDF pose" % name)
            return
        if self.state in (STATE_PLANNING, STATE_EXECUTING) or self.task["running"]:
            self.set_result(cmd, "error", "robot is moving — teach only when stationary")
            return
        pos = self.js.positions(ARM_JOINTS)
        if pos is None:
            self.set_result(cmd, "error", "no joint states")
            return
        if max(abs(v) for v in self.js.velocities(ARM_JOINTS)) > self.vel_tol * 5:
            self.set_result(cmd, "error", "joints still moving")
            return
        tcp = self.current_tcp()
        if tcp is None:
            self.set_result(cmd, "error", "TCP pose not available from MoveIt")
            return
        if self.collision.get("valid") is False:
            self.set_result(cmd, "error", "current state is in collision: %s" % self.collision.get("contacts"))
            return
        rec = {"joints_deg": [round(v, 3) for v in rad2deg(pos)], "tcp": tcp, "stamp": time.time()}
        with self.lock:
            self.taught[name] = rec
            self.teach_last = dict(rec, name=name)
        try:
            self._save_taught()
        except Exception as exc:
            self.set_result(cmd, "error", "pose kept in memory but file write failed: %s" % exc)
            self.publish_info()
            return
        rospy.loginfo("pendant_bridge: TEACH %s joints=%s tcp=%s", name, rec["joints_deg"],
                      [tcp["x"], tcp["y"], tcp["z"]])
        self.publish_info()
        self.set_result(cmd, "ok", "pose %s saved (%s)" % (name, os.path.basename(self.taught_file)))

    def delete_taught(self, cmd):
        name = str(cmd.get("name", "")).strip().upper()
        if name not in self.taught:
            self.set_result(cmd, "error", "no taught pose '%s'" % name)
            return
        with self.lock:
            del self.taught[name]
        try:
            self._save_taught()
        except Exception as exc:
            rospy.logwarn("pendant_bridge: saving taught poses failed: %s", exc)
        self.publish_info()
        self.set_result(cmd, "ok", "pose %s deleted" % name)

    # ------------------------------------------------------------- helpers
    def set_state(self, state, error=None):
        with self.lock:
            if self.estop and state != STATE_ESTOP:
                state = STATE_ESTOP        # latched simulation E-STOP is never hidden by a later FAILED/STOPPED/IDLE
            self.state = state
            if error is not None:
                self.last_error = error
        if error:
            rospy.logwarn("pendant_bridge: %s — %s", state, error)
        else:
            rospy.loginfo("pendant_bridge: state %s", state)

    def set_result(self, cmd, result, message=""):
        with self.lock:
            self.last_cmd = {"id": cmd.get("id"), "cmd": cmd.get("cmd"), "result": result, "message": message}

    def check_stop(self):
        if self.stop_evt.is_set():
            raise StopRequested()

    def _now(self):
        """Execution clock (seconds): simulation time when /use_sim_time is set, otherwise wall time."""
        return rospy.get_time() if self.use_sim_time else time.time()

    def _rtf_loop(self):
        while not rospy.is_shutdown():
            try:
                self._sample_rtf(None)
            except Exception:
                pass
            time.sleep(1.0)

    def _sample_rtf(self, _evt):
        """Real-time factor = sim seconds elapsed / wall seconds elapsed, sampled about once per second."""
        wall, sim = time.time(), rospy.get_time()
        if self._rtf_sample is not None:
            dw = wall - self._rtf_sample[0]
            if dw >= 0.5:
                self.rtf = round((sim - self._rtf_sample[1]) / dw, 2) if self.use_sim_time else 1.0
                self._rtf_sample = (wall, sim)
        else:
            self._rtf_sample = (wall, sim)

    def speed_factor(self):
        return max(0.01, min(1.0, self.speed_pct / 100.0))

    def validate_joint_limits(self, joints_rad):
        for n, v in zip(ARM_JOINTS, joints_rad):
            lim = self.limits.get(n)
            if lim and not (lim["lower"] - 1e-6 <= v <= lim["upper"] + 1e-6):
                return "%s=%.1f° outside [%.1f°, %.1f°]" % (
                    n, math.degrees(v), math.degrees(lim["lower"]), math.degrees(lim["upper"]))
        return None

    def current_tcp(self):
        try:
            ps = self.group.get_current_pose(EEF_LINK)
            p, o = ps.pose.position, ps.pose.orientation
            rpy = tft.euler_from_quaternion([o.x, o.y, o.z, o.w])
            return {"x": round(p.x, 4), "y": round(p.y, 4), "z": round(p.z, 4),
                    "roll": round(math.degrees(rpy[0]), 2), "pitch": round(math.degrees(rpy[1]), 2),
                    "yaw": round(math.degrees(rpy[2]), 2),
                    "q": [round(o.x, 6), round(o.y, 6), round(o.z, 6), round(o.w, 6)],
                    "frame": ps.header.frame_id}
        except Exception:
            return None

    # ------------------------------------------------------------ planning
    def _moveit_plan(self, planner_label):
        """Run group.plan() and normalise the result across MoveIt versions. Planning only; nothing is executed."""
        self.group.set_start_state_to_current_state()
        self.group.set_max_velocity_scaling_factor(self.speed_factor())
        self.group.set_max_acceleration_scaling_factor(self.speed_factor())
        with self.lock:
            self.plan_info["attempted"] = True
            cmd_id = self.plan_info.get("cmd_id")
        t0 = time.time()
        res = self.group.plan()
        dt = time.time() - t0
        if isinstance(res, tuple):          # Noetic: (success, trajectory, planning_time, error_code)
            ok, traj, _ptime, err = res
            code = err.val if hasattr(err, "val") else int(err)
        else:                               # older API: RobotTrajectory
            traj = res
            ok = len(traj.joint_trajectory.points) > 0
            code = MoveItErrorCodes.SUCCESS if ok else MoveItErrorCodes.PLANNING_FAILED
        self.group.clear_pose_targets()
        pts = traj.joint_trajectory.points if traj is not None else []
        with self.lock:
            self.plan_info = {"cmd_id": cmd_id, "attempted": True,
                              "ok": bool(ok), "planner": planner_label, "time_s": round(dt, 3),
                              "traj_duration_s": round(pts[-1].time_from_start.to_sec(), 3) if pts else None,
                              "points": len(pts), "fraction": None,
                              "error_code": MOVEIT_ERROR_NAMES.get(code, str(code))}
        return bool(ok) and len(pts) > 0, traj

    def plan_and_execute(self, prepare, planner_label):
        """prepare(): sets the MoveIt target. Plans, executes, verifies."""
        self.set_state(STATE_PLANNING)
        self.check_stop()
        try:
            prepare()
        except Exception as exc:
            self.set_state(STATE_FAILED, "target rejected: %s" % exc)
            return False
        ok, traj = self._moveit_plan(planner_label)
        if not ok:
            self.set_state(STATE_FAILED, "planning failed (%s)" % self.plan_info["error_code"])
            return False
        return self.execute_trajectory(traj.joint_trajectory)

    def execute_trajectory(self, jt):
        """Send a JointTrajectory to the arm controller via FollowJointTrajectory and verify arrival."""
        self.check_stop()
        if not jt.points:
            self.set_state(STATE_FAILED, "empty trajectory")
            return False
        duration = jt.points[-1].time_from_start.to_sec()
        timeout = duration + max(2.0, 0.25 * duration)
        target = dict(zip(jt.joint_names, jt.points[-1].positions))
        with self.lock:
            self.target_rad = {n: target.get(n, self.target_rad.get(n) if self.target_rad else 0.0)
                               for n in ARM_JOINTS}
            self.exec_info = {"cmd_id": self.exec_info.get("cmd_id"), "attempted": True,
                              "goal_status": "ACTIVE", "elapsed_s": 0.0, "timeout_s": round(timeout, 2)}
        self.set_state(STATE_EXECUTING)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = jt
        goal.trajectory.header.stamp = rospy.Time(0)
        self.fjt.send_goal(goal)
        t0 = self._now()                      # execution clock (sim time under Gazebo)
        w0 = time.time()
        wall_cap = max(30.0, self.wall_cap_factor * timeout)
        status = None
        while not rospy.is_shutdown():
            if self.stop_evt.is_set():
                self.fjt.cancel_goal()
                with self.lock:
                    self.exec_info["goal_status"] = "PREEMPTED"
                raise StopRequested()
            status = self.fjt.get_state()
            elapsed = self._now() - t0
            with self.lock:
                self.exec_info["elapsed_s"] = round(elapsed, 2)
            if status in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED,
                          GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST):
                break
            if elapsed > timeout:
                self.fjt.cancel_goal()
                with self.lock:
                    self.exec_info["goal_status"] = "TIMEOUT"
                self.set_state(STATE_FAILED, "execution timeout after %.1fs sim time (limit %.1fs, RTF %s)" % (
                    elapsed, timeout, self.rtf))
                return False
            if time.time() - w0 > wall_cap:
                self.fjt.cancel_goal()
                with self.lock:
                    self.exec_info["goal_status"] = "TIMEOUT"
                self.set_state(STATE_FAILED, "wall-clock cap %.0fs exceeded (simulation paused or RTF too low; sim elapsed %.1fs)" % (
                    wall_cap, elapsed))
                return False
            time.sleep(0.02)
        name = GOAL_STATUS_NAMES.get(status, str(status))
        with self.lock:
            self.exec_info["goal_status"] = name
        if status != GoalStatus.SUCCEEDED:
            self.set_state(STATE_FAILED, "controller goal %s" % name)
            return False
        ok, err = self.wait_for_target(target, timeout=1.5)
        if not ok:
            self.set_state(STATE_FAILED, "target not reached (max err %.2f°)" % math.degrees(err))
            return False
        self.set_state(STATE_REACHED)
        return True

    def wait_for_target(self, target, timeout, tol=None):
        """True when all joints in target are within tolerance and (nearly) stopped for settle_time."""
        tol = self.joint_tol if tol is None else tol
        names = list(target.keys())
        goal = [target[n] for n in names]
        t0 = self._now()                      # execution clock (sim time under Gazebo)
        w0 = time.time()
        wall_cap = max(10.0, self.wall_cap_factor * timeout)
        settled_since = None
        max_err = float("inf")
        while not rospy.is_shutdown():
            if self.stop_evt.is_set():
                raise StopRequested()
            pos = self.js.positions(names)
            vel = self.js.velocities(names)
            if pos is not None:
                max_err = max(abs(p - g) for p, g in zip(pos, goal))
                max_vel = max(abs(v) for v in vel)
                if max_err < tol and max_vel < self.vel_tol:
                    settled_since = settled_since or self._now()
                    if self._now() - settled_since >= self.settle_time:
                        return True, max_err
                else:
                    settled_since = None
            if self._now() - t0 > timeout or time.time() - w0 > wall_cap:
                return False, max_err
            time.sleep(0.02)
        return False, max_err

    # ------------------------------------------------------------ commands
    def on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("pendant_bridge: invalid JSON command: %s", msg.data[:80])
            return
        name = cmd.get("cmd")
        rospy.loginfo("pendant_bridge: cmd %s", json.dumps(cmd))

        # --- immediate commands (allowed under E-STOP) ---
        if name == "estop":
            if cmd.get("active", True):
                self.do_estop()
                self.set_result(cmd, "ok", "simulation E-STOP active")
            elif self.busy:
                # the worker is still inside a (non-interruptible) planning call or finishing the stop:
                # keep the latch until it has actually stopped, otherwise the pending trajectory could run
                self.set_result(cmd, "rejected", "worker still stopping — retry")
            else:
                self.reset_estop()
                self.set_result(cmd, "ok", "E-STOP reset")
            return
        if name == "resolve_pose":
            # read-only: report exactly what move_named would command for this name (no motion)
            source, joints_deg, srdf = self._resolve_named(cmd.get("name", ""))
            if source is None:
                self.set_result(cmd, "error", "unknown pose '%s' (not in named_poses / taught / SRDF)" % cmd.get("name"))
            else:
                try:
                    joints_deg = self._resolved_joints_deg(cmd.get("name", ""))
                    self.set_result(cmd, "ok", json.dumps({"name": str(cmd.get("name", "")).upper(), "source": source,
                                                          "joints_deg": [round(float(v), 3) for v in joints_deg]}))
                except Exception as exc:
                    self.set_result(cmd, "error", "resolve_pose failed: %s" % exc)
            return
        if name == "stop":
            self.do_stop()
            self.set_result(cmd, "ok", "stopped")
            return
        if name == "set_speed":
            self.speed_pct = float(max(1, min(100, cmd.get("speed", self.speed_pct))))
            self.set_result(cmd, "ok", "speed %d%%" % self.speed_pct)
            return
        if name == "get_info":
            self.publish_info()
            self.set_result(cmd, "ok")
            return
        if name == "teach":
            self.do_teach(cmd)
            return
        if name == "delete_taught":
            self.delete_taught(cmd)
            return
        if name == "reload_taught":
            # re-read the scenario's taught poses file (e.g. after find_task_poses.py --write-taught); read-only, no motion
            with self.lock:
                self.taught = self._load_taught()
            self.publish_info()
            self.set_result(cmd, "ok", "taught poses reloaded: %s" % ", ".join(sorted(self.taught.keys())))
            return
        if name in ("attach", "detach", "reset_scene"):
            # Gazebo object handling (no robot motion). Allowed under E-STOP; refused while a job is running.
            if self.busy:
                self.set_result(cmd, "rejected", "bridge busy — wait for the current command")
                return
            self.step_note = None
            handler = {"attach": self.attach_object, "detach": self.detach_object, "reset_scene": self.scene_reset}[name]
            ok = handler(cmd.get("object", next(iter(self.objects), "cup")))
            self.set_result(cmd, "ok" if ok else "error", self.step_note if ok else self.last_error)
            return
        if name == "reload_workcell":
            self.add_workcell_objects()
            self.publish_info()
            self.set_result(cmd, "ok", "planning scene: %s" % self.workcell_loaded)
            return

        # --- queued motion commands ---
        if self.estop:
            self.set_result(cmd, "rejected", "simulation E-STOP active")
            return
        if name in ("move_joints", "move_named", "move_pose", "cartesian_jog", "gripper", "run_task"):
            if self.task["running"]:
                self.set_result(cmd, "rejected", "task running — stop it first")
                return
            self.set_result(cmd, "queued")     # before enqueueing: an instant rejection must not be overwritten by "queued"
            self.jobs.put(cmd)
        else:
            self.set_result(cmd, "rejected", "unknown command '%s'" % name)

    def _drain_jobs(self):
        try:
            while True:
                self.jobs.get_nowait()
        except queue.Empty:
            pass

    def _worker(self):
        while not rospy.is_shutdown():
            try:
                cmd = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            if self.estop:
                self.set_result(cmd, "rejected", "simulation E-STOP active")
                continue
            self.stop_evt.clear()
            with self.lock:                      # per-command tracking: fresh plan/exec snapshots tagged with this cmd id
                self.plan_info = self._empty_plan(cmd.get("id"))
                self.exec_info = self._empty_exec(cmd.get("id"))
                self.last_error = ""
            self.busy = True
            try:
                ok = self.dispatch(cmd)
                self.set_result(cmd, "ok" if ok else "error", "" if ok else self.last_error)
            except StopRequested:
                self.set_state(STATE_ESTOP if self.estop else STATE_STOPPED, "stopped by operator")
                self.set_result(cmd, "stopped", "stopped by operator")
            except Exception as exc:
                rospy.logerr("pendant_bridge: %s\n%s", exc, traceback.format_exc())
                self.set_state(STATE_FAILED, str(exc))
                self.set_result(cmd, "error", str(exc))
            finally:
                self.busy = False

    def dispatch(self, cmd):
        name = cmd["cmd"]
        if "speed" in cmd:
            self.speed_pct = float(max(1, min(100, cmd["speed"])))
        if name == "move_joints":
            return self.move_joints_deg(cmd.get("deg") or cmd.get("joints_deg"))
        if name == "move_named":
            return self.move_named(cmd.get("name", ""))
        if name == "move_pose":
            return self.move_pose(cmd.get("xyz"), cmd.get("rpy_deg"))
        if name == "cartesian_jog":
            return self.cartesian_jog(cmd.get("axis"), float(cmd.get("delta", 0.0)))
        if name == "gripper":
            return self.gripper_cmd(cmd.get("action", "open"), cmd.get("angle_deg"), cmd.get("time"))
        if name == "run_task":
            return self.run_task(cmd.get("task", "barista"))
        return False

    # ---- motion primitives -------------------------------------------------
    def move_joints_deg(self, deg, label="ompl/joint"):
        if not deg or len(deg) != 6:
            self.set_state(STATE_FAILED, "move_joints needs 6 values")
            return False
        rad = deg2rad(deg)
        err = self.validate_joint_limits(rad)
        if err:
            self.set_state(STATE_FAILED, "joint limit: " + err)
            return False
        return self.plan_and_execute(lambda: self.group.set_joint_value_target(rad), label)

    def move_named(self, name):
        """Resolution order (shared with _resolve_named): named_poses (yaml) -> taught poses -> SRDF group states."""
        source, joints_deg, srdf = self._resolve_named(name)
        if source == "named_poses":
            return self.move_joints_deg(joints_deg, "ompl/named:" + str(name).upper())
        if source == "taught":
            return self.move_joints_deg(joints_deg, "ompl/taught:" + str(name).upper())
        if source == "srdf":
            return self.plan_and_execute(lambda: self.group.set_named_target(srdf), "ompl/srdf:" + srdf)
        self.set_state(STATE_FAILED, "unknown pose '%s' (not in named_poses / taught / SRDF)" % name)
        return False

    def _pose_from(self, xyz, rpy_deg=None, q=None):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
        if q is None:
            q = tft.quaternion_from_euler(*deg2rad(rpy_deg))
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [float(v) for v in q]
        return pose

    def move_pose(self, xyz, rpy_deg):
        if not xyz or not rpy_deg:
            self.set_state(STATE_FAILED, "move_pose needs xyz and rpy_deg")
            return False
        pose = self._pose_from(xyz, rpy_deg)
        return self.plan_and_execute(lambda: self.group.set_pose_target(pose, EEF_LINK), "ompl/pose(IK)")

    def _cartesian_to_pose(self, target, label, min_fraction):
        """Straight-line TCP motion to `target` (base_link): compute_cartesian_path -> flip guard -> retime -> FJT."""
        self.set_state(STATE_PLANNING)
        self.check_stop()
        self.group.set_start_state_to_current_state()
        with self.lock:
            self.plan_info["attempted"] = True
            cmd_id = self.plan_info.get("cmd_id")
        t0 = time.time()
        # Keyword form: the installed moveit_commander has no positional jump_threshold argument
        # (a positional 3rd/4th value is taken as path_constraints). Configuration flips are caught
        # by the cartesian_max_joint_jump_rad guard below instead.
        plan, fraction = self.group.compute_cartesian_path([target], self.cart_eef_step, avoid_collisions=True)
        dt = time.time() - t0
        pts = plan.joint_trajectory.points
        jump = 0.0
        for a, b in zip(pts, pts[1:]):
            jump = max(jump, max(abs(x - y) for x, y in zip(a.positions, b.positions)))
        if pts and jump <= self.cart_max_jump:
            try:
                plan = self.group.retime_trajectory(self.robot.get_current_state(), plan,
                                                    velocity_scaling_factor=self.speed_factor(),
                                                    acceleration_scaling_factor=self.speed_factor(),
                                                    algorithm="time_optimal_trajectory_generation")
                pts = plan.joint_trajectory.points
            except Exception as exc:
                rospy.logwarn("pendant_bridge: retime failed (%s), using raw cartesian timing", exc)
        with self.lock:
            self.plan_info = {"cmd_id": cmd_id, "attempted": True,
                              "ok": bool(pts) and fraction >= min_fraction and jump <= self.cart_max_jump,
                              "planner": label, "time_s": round(dt, 3),
                              "traj_duration_s": round(pts[-1].time_from_start.to_sec(), 3) if pts else None,
                              "points": len(pts), "fraction": round(fraction, 3), "error_code": None}
        if not pts or fraction < min_fraction:
            self.set_state(STATE_FAILED, "cartesian path only %.0f%% feasible (reach/collision, need %.0f%%)" % (
                fraction * 100, min_fraction * 100))
            return False
        if jump > self.cart_max_jump:
            self.set_state(STATE_FAILED, "cartesian path has a configuration flip (joint jump %.2f rad)" % jump)
            return False
        return self.execute_trajectory(plan.joint_trajectory)

    def cartesian_offset(self, dxyz, drpy_deg=(0, 0, 0), label="cartesian", min_fraction=None):
        """Straight-line TCP move by an offset (m in base_link; rotation deg in tool frame)."""
        start = self.group.get_current_pose(EEF_LINK).pose
        q0 = [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w]
        q = tft.quaternion_multiply(q0, tft.quaternion_from_euler(*deg2rad(drpy_deg)))
        target = self._pose_from([start.position.x + dxyz[0], start.position.y + dxyz[1], start.position.z + dxyz[2]], q=q)
        return self._cartesian_to_pose(target, label, self.cart_min_fraction if min_fraction is None else min_fraction)

    def cartesian_to(self, pose_name, dxyz=(0, 0, 0), min_fraction=1.0, label=None):
        """Straight-line TCP move to a TAUGHT pose's stored TCP (+ optional base_link offset)."""
        key = str(pose_name).upper()
        rec = self.taught.get(key)
        if rec is None:
            self.set_state(STATE_FAILED, "cartesian_to: '%s' is not a taught pose" % key)
            return False
        tcp = rec["tcp"]
        target = self._pose_from([tcp["x"] + dxyz[0], tcp["y"] + dxyz[1], tcp["z"] + dxyz[2]], q=tcp["q"])
        return self._cartesian_to_pose(target, label or ("cartesian_to:" + key), min_fraction)

    def cartesian_jog(self, axis, delta):
        axis = (axis or "").lower()
        dxyz = [0.0, 0.0, 0.0]
        drpy = [0.0, 0.0, 0.0]
        if axis in ("x", "y", "z"):
            dxyz["xyz".index(axis)] = delta           # metres
        elif axis in ("rx", "ry", "rz"):
            drpy["xyz".index(axis[1])] = delta        # degrees
        else:
            self.set_state(STATE_FAILED, "unknown axis '%s'" % axis)
            return False
        return self.cartesian_offset(dxyz, drpy, "cartesian_jog/" + axis)

    # ---- gripper -----------------------------------------------------------
    def gripper_cmd(self, action, angle_deg=None, duration=None):
        if self.gripper_srv is None:
            self.set_state(STATE_FAILED, "gripper service unavailable")
            return False
        if action == "open":
            angle = self.gripper_open_deg
        elif action == "close":
            angle = self.gripper_close_deg
        elif action == "angle" and angle_deg is not None:
            angle = float(angle_deg)
        else:
            self.set_state(STATE_FAILED, "gripper action must be open|close|angle")
            return False
        duration = float(duration or self.gripper_time)
        self.check_stop()
        self.set_state(STATE_EXECUTING)
        with self.lock:
            self.gripper["cmd_deg"] = angle
            self.gripper["state"] = "MOVING"
            self.gripper["reached"] = None
        resp = self.gripper_srv(angle, duration)
        if not resp.success:
            self.set_state(STATE_FAILED, "gripper service: %s" % resp.message)
            return False
        # Verify: left joints are commanded to +angle, right joints to -angle (RosGripperControl.py)
        target = {"grip_joint_1_left": math.radians(angle), "grip_joint_1_right": -math.radians(angle)}
        ok, err = self.wait_for_target(target, timeout=duration + 1.5, tol=math.radians(self.gripper_tol_deg))
        with self.lock:
            self.gripper["reached"] = ok
        if not ok:
            self.set_state(STATE_FAILED, "gripper did not reach %.1f° (err %.1f°)" % (angle, math.degrees(err)))
            return False
        if action == "close":
            rospy.loginfo("pendant_bridge: gripper CLOSED at %.1f° — %s", angle, GRASP_NOTE)
        else:
            rospy.loginfo("pendant_bridge: gripper %s reached %.1f° (position verified)", action, angle)
        self.set_state(STATE_REACHED)
        return True

    def gripper_status(self):
        pos = self.js.positions(["grip_joint_1_left"])
        if pos is None:
            return
        ang = math.degrees(pos[0])
        tol = self.gripper_tol_deg
        with self.lock:
            self.gripper["angle_deg"] = round(ang, 1)
            if self.gripper["state"] != "MOVING" or self.gripper["reached"] is not None:
                if abs(ang - self.gripper_open_deg) < tol:
                    self.gripper["state"] = "OPEN"
                elif abs(ang - self.gripper_close_deg) < tol:
                    self.gripper["state"] = "CLOSED"
                else:
                    self.gripper["state"] = "UNKNOWN"

    # ---- task runner (steps from ~tasks, see config/scenarios/<scenario>.yaml) -----
    def run_task(self, task_name):
        task = self.tasks.get(task_name)
        if not task:
            self.set_state(STATE_FAILED, "no task '%s' loaded" % task_name)
            return False
        req = self.task_requirements(task_name)
        if req["missing"]:
            self.set_state(STATE_FAILED, "task '%s' needs taught poses: %s (use Teach)" % (
                task_name, ", ".join(req["missing"])))
            return False
        steps = task.get("steps", [])
        saved_speed = self.speed_pct
        with self.lock:
            self.task.update({"name": task_name, "running": True, "step_index": -1, "step_count": len(steps),
                              "step_name": None, "result": None, "message": "",
                              "steps": [{"name": s.get("name", s.get("type")), "type": s.get("type"),
                                         "state": "pending", "t_s": None, "plan_s": None, "note": None} for s in steps],
                              "cycle_time_s": 0.0})
            self.task_t0 = time.time()
            self.task_t0_sim = rospy.get_time()
        rospy.loginfo("pendant_bridge: task '%s' started (%d steps) — %s", task_name, len(steps), GRASP_NOTE)
        ok = False
        try:
            for i, step in enumerate(steps):
                self.check_stop()
                with self.lock:
                    self.task["step_index"] = i
                    self.task["step_name"] = step.get("name")
                    self.task["steps"][i]["state"] = "active"
                self.step_outcome = None
                self.step_note = None
                t_step = time.time()
                ok = self.run_step(step)
                with self.lock:
                    if ok and self.step_outcome == "skipped":
                        self.task["steps"][i]["state"] = "skipped"
                    else:
                        self.task["steps"][i]["state"] = "done" if ok else "failed"
                    self.task["steps"][i]["note"] = self.step_note
                    self.task["steps"][i]["t_s"] = round(time.time() - t_step, 2)
                    if step.get("type") in ("named", "named_if_needed", "joint", "pose", "cartesian_to", "cartesian_offset") \
                            and self.plan_info.get("attempted"):
                        self.task["steps"][i]["plan_s"] = self.plan_info.get("time_s")
                if not ok:
                    break      # failed step: the task stops here; no automatic recovery/HOME motion
            with self.lock:
                self.task["result"] = "ok" if ok else "failed"
                self.task["message"] = "" if ok else self.last_error
        except StopRequested:
            with self.lock:
                idx = self.task["step_index"]
                if 0 <= idx < len(self.task["steps"]):
                    self.task["steps"][idx]["state"] = "failed"
                self.task["result"] = "stopped"
                self.task["message"] = "stopped by operator"
            raise
        finally:
            self.speed_pct = saved_speed
            obj_now = self.object_status()           # outside the lock (object_pose_base locks itself)
            with self.lock:
                self.task["running"] = False
                self.task["last_cycle_time_s"] = round(time.time() - self.task_t0, 2)          # wall clock
                self.task["last_cycle_time_sim_s"] = round(rospy.get_time() - self.task_t0_sim, 2)   # simulation clock
                self.task["cycle_time_s"] = self.task["last_cycle_time_s"]
                self.task["count"] += 1
                if self.task["result"] is None:
                    self.task["result"] = "error"
                    self.task["message"] = self.task["message"] or self.last_error
                row = self._history_row(task_name, obj_now)
                self.history.append(row)
            self._append_history(row)
            rospy.loginfo("pendant_bridge: task '%s' finished: %s (%.1fs wall / %.1fs sim)", task_name,
                          self.task["result"], self.task["last_cycle_time_s"], self.task["last_cycle_time_sim_s"])
        return ok

    def _history_row(self, task_name, obj_now):
        """One task-history record (called with self.lock held)."""
        t = self.task
        failed = [(i, s) for i, s in enumerate(t["steps"]) if s.get("state") == "failed"]
        obj_name = next(iter(obj_now or {}), None)             # the scenario's (first) handled object, if any
        cup = (obj_now or {}).get(obj_name) or {}
        return {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "scenario": self.scenario.get("name"),
                "task": task_name, "result": t.get("result") or "error", "object": obj_name,
                "cycle_time_s": t.get("last_cycle_time_s"), "cycle_time_sim_s": t.get("last_cycle_time_sim_s"),
                "speed_pct": self.speed_pct, "steps_total": len(t["steps"]),
                "steps_done": sum(1 for s in t["steps"] if s.get("state") in ("done", "skipped")),
                "failure_step": ("%d:%s" % (failed[0][0] + 1, failed[0][1].get("name"))) if failed else None,
                "message": t.get("message") or "", "object_at_place": cup.get("at_place"),
                "object_attached": cup.get("attached"), "rtf": self.rtf,
                "steps": "|".join("%s:%s:%s" % (s.get("name"), s.get("state"), s.get("t_s")) for s in t["steps"])}

    def _append_history(self, row):
        """Append one record to the task history CSV (header written when the file is created)."""
        if not self.history_file:
            return
        try:
            d = os.path.dirname(self.history_file)
            if d:
                os.makedirs(d, exist_ok=True)
            new_file = not os.path.isfile(self.history_file)
            with open(self.history_file, "a", newline="") as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(HISTORY_COLS)
                w.writerow([row.get(c) for c in HISTORY_COLS])
        except Exception as exc:
            rospy.logwarn("pendant_bridge: task history write failed (%s): %s", self.history_file, exc)

    def run_step(self, step):
        t = step.get("type")
        if "speed" in step:
            self.speed_pct = float(max(1, min(100, step["speed"])))
        if t == "state_check":
            return self.state_check(step)
        if t == "named_if_needed":
            return self.named_if_needed(step)
        if t == "scene_reset":
            return self.scene_reset(step.get("object", "cup"))
        if t == "attach":
            return self.attach_object(step.get("object", "cup"))
        if t == "detach":
            return self.detach_object(step.get("object", "cup"))
        if t == "verify_tcp":
            return self.verify_tcp(step.get("pose", ""), float(step.get("tol_m", 0.01)), float(step.get("tol_deg", 3.0)))
        if t == "verify_object":
            return self.verify_object(step.get("object", "cup"), step.get("expect", "at_place"), step.get("tol_m"))
        if t == "named":
            return self.move_named(step.get("pose", step.get("name_pose", "")))
        if t == "joint":
            return self.move_joints_deg(step["joints_deg"])
        if t == "pose":
            return self.move_pose(step["xyz"], step["rpy_deg"])
        if t == "cartesian_to":
            return self.cartesian_to(step["pose"], step.get("dxyz", [0, 0, 0]),
                                     float(step.get("min_fraction", 1.0)), "cartesian_to:" + step.get("name", ""))
        if t == "cartesian_offset":
            return self.cartesian_offset(step.get("dxyz", [0, 0, 0]), step.get("drpy_deg", [0, 0, 0]),
                                         "cartesian/" + step.get("name", "offset"), step.get("min_fraction"))
        if t == "gripper":
            return self.gripper_cmd(step.get("action", "open"), step.get("angle_deg"), step.get("time"))
        if t == "wait":
            dur = float(step.get("seconds", 0.5))
            t0 = time.time()
            while time.time() - t0 < dur:
                self.check_stop()
                time.sleep(0.05)
            return True
        self.set_state(STATE_FAILED, "unknown step type '%s'" % t)
        return False

    def state_check(self, step):
        """Task start gate (no motion): joint states fresh, arm stationary, current state collision-free."""
        self.check_stop()
        pos = self.js.positions(ARM_JOINTS)
        age = time.time() - self.js.wall_stamp
        if pos is None or age > 1.0:
            self.set_state(STATE_FAILED, "no fresh joint states at task start (age %.1fs)" % age)
            return False
        max_vel = max(abs(v) for v in self.js.velocities(ARM_JOINTS))
        if max_vel > self.vel_tol * 5:
            self.set_state(STATE_FAILED, "arm moving at task start (%.3f rad/s)" % max_vel)
            return False
        try:
            valid, contacts = self._validity_now()
        except Exception as exc:
            self.set_state(STATE_FAILED, "state validity check failed: %s" % exc)
            return False
        if valid is False:
            self.set_state(STATE_FAILED, "start state in collision: %s" % ", ".join(contacts))
            return False
        if valid is not None:
            with self.lock:
                self.collision = {"valid": valid, "contacts": contacts}
        self.step_note = "start state ok (stationary, %s)" % (
            "collision-free" if valid else "validity service unavailable — not checked")
        rospy.loginfo("pendant_bridge: STATE_CHECK — %s", self.step_note)
        return True

    def named_if_needed(self, step):
        """Move to `pose` only if needed. Skipped when the arm is already there, or when a direct
        collision-checked plan to `unless_reachable` exists (PLANNING ONLY — that plan is discarded,
        never executed; the following task step plans and executes its own move)."""
        pose = step.get("pose", "")
        target_deg = self._resolved_joints_deg(pose)
        if target_deg is None:
            self.set_state(STATE_FAILED, "unknown pose '%s' (not in named_poses / taught / SRDF)" % pose)
            return False
        label = str(pose).upper()
        cur = self.js.positions(ARM_JOINTS)
        if cur is not None and max(abs(c - g) for c, g in zip(cur, deg2rad(target_deg))) < self.joint_tol:
            self.step_outcome = "skipped"
            self.step_note = "already at %s" % label
            rospy.loginfo("pendant_bridge: %s skipped — %s", label, self.step_note)
            return True
        nxt = step.get("unless_reachable")
        if nxt:
            next_deg = self._resolved_joints_deg(nxt)
            if next_deg is None:
                self.step_note = "%s required: '%s' unknown" % (label, nxt)
            else:
                self.set_state(STATE_PLANNING)
                self.check_stop()
                ok = False
                try:
                    self.group.set_joint_value_target(deg2rad(next_deg))
                    ok, _traj = self._moveit_plan("ompl/check:%s" % str(nxt).upper())   # planning only, trajectory discarded
                except Exception as exc:
                    rospy.logwarn("pendant_bridge: feasibility plan to %s failed: %s", nxt, exc)
                    self.group.clear_pose_targets()
                if ok:
                    self.step_outcome = "skipped"
                    self.step_note = "%s skipped: direct collision-checked plan to %s ok (%.2f s)" % (
                        label, str(nxt).upper(), self.plan_info.get("time_s") or 0.0)
                    self.set_state(STATE_IDLE)
                    rospy.loginfo("pendant_bridge: %s", self.step_note)
                    return True
                self.step_note = "%s required: direct plan to %s failed (%s)" % (
                    label, str(nxt).upper(), self.plan_info.get("error_code"))
                rospy.loginfo("pendant_bridge: %s", self.step_note)
        return self.move_named(pose)

    # ---- Gazebo objects: scene reset, kinematic attach/detach, measured checks (SIMULATION ONLY) ----
    def _init_gazebo_objects(self):
        if not self.objects:
            return
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1)
        self._link_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self._on_link_states, queue_size=1)
        try:
            rospy.wait_for_service(SET_MODEL_STATE_SERVICE, timeout=5.0)
            self.set_state_srv = rospy.ServiceProxy(SET_MODEL_STATE_SERVICE, SetModelState)
        except rospy.ROSException:
            rospy.logwarn("pendant_bridge: %s not available — scene_reset steps will fail", SET_MODEL_STATE_SERVICE)
        if not HAVE_ATTACHER:
            rospy.logwarn("pendant_bridge: gazebo_ros_link_attacher is not installed — attach/detach steps will fail "
                          "(git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git into catkin_ws/src)")
            return
        try:
            rospy.wait_for_service(ATTACH_SERVICE, timeout=5.0)
            self.attach_srv = rospy.ServiceProxy(ATTACH_SERVICE, Attach)
            self.detach_srv = rospy.ServiceProxy(DETACH_SERVICE, Attach)
        except rospy.ROSException:
            rospy.logwarn("pendant_bridge: %s not available (is the ros_link_attacher_plugin loaded in the world?)", ATTACH_SERVICE)

    def _on_model_states(self, msg):
        now = time.time()
        if now - self.model_poses_wall < 0.05:      # topic is published at the physics rate; 20 Hz is plenty
            return
        with self.lock:
            for n, p in zip(msg.name, msg.pose):
                self.model_poses[n] = (p.position.x, p.position.y, p.position.z)
            self.model_poses_wall = now

    def _on_link_states(self, msg):
        self.gazebo_links = set(msg.name)
        try:
            self._link_sub.unregister()             # one snapshot is enough (link names do not change)
        except Exception:
            pass

    def object_cfg(self, name):
        return self.objects.get(str(name))

    def object_pose_base(self, name):
        """Object centre in base_link coordinates from /gazebo/model_states, or None if unknown/stale."""
        cfg = self.object_cfg(name)
        if not cfg:
            return None
        with self.lock:
            p = self.model_poses.get(cfg.get("gazebo_model", name))
            fresh = (time.time() - self.model_poses_wall) < 1.0
        if p is None or not fresh:
            return None
        return [p[0], p[1], p[2] - self.gazebo_z_offset]

    @staticmethod
    def _near(a, b, tol):
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= tol and abs(a[2] - b[2]) <= tol

    def gripper_gazebo_link(self):
        """Gazebo lumps fixed-joint children into their parent, so the gripper usually lives in link_6."""
        for cand in self.gripper_link_candidates:
            if "%s::%s" % (self.robot_model, cand) in self.gazebo_links:
                return cand
        return self.gripper_link_candidates[-1]

    def object_status(self):
        out = {}
        for name, cfg in self.objects.items():
            pb = self.object_pose_base(name)
            tol = float(cfg.get("tolerance_m", 0.03))
            st = {"pose_base": [round(v, 4) for v in pb] if pb else None,
                  "attached": name in self.attached, "attached_to": self.attached.get(name),
                  "at_start": self._near(pb, cfg["start_xyz_base"], tol) if (pb and cfg.get("start_xyz_base")) else None,
                  "at_place": self._near(pb, cfg["place_xyz_base"], tol) if (pb and cfg.get("place_xyz_base")) else None,
                  "attacher_available": self.attach_srv is not None,
                  "set_state_available": self.set_state_srv is not None}
            out[name] = st
        return out

    def scene_reset(self, name):
        """Put the object back at its start pose (detaching first if needed). No robot motion."""
        cfg = self.object_cfg(name)
        if not cfg or not cfg.get("start_xyz_base"):
            self.set_state(STATE_FAILED, "unknown object '%s' (see ~objects)" % name)
            return False
        if self.set_state_srv is None:
            self.set_state(STATE_FAILED, "%s unavailable — cannot reset '%s'" % (SET_MODEL_STATE_SERVICE, name))
            return False
        if name in self.attached and not self.detach_object(name):
            return False
        req = SetModelStateRequest()
        ms = req.model_state
        ms.model_name = cfg.get("gazebo_model", name)
        ms.reference_frame = "world"
        x, y, z = [float(v) for v in cfg["start_xyz_base"]]
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = x, y, z + self.gazebo_z_offset
        q = tft.quaternion_from_euler(*deg2rad(cfg.get("start_rpy_deg", [0, 0, 0])))
        ms.pose.orientation.x, ms.pose.orientation.y, ms.pose.orientation.z, ms.pose.orientation.w = q
        try:
            resp = self.set_state_srv(req)
        except Exception as exc:
            self.set_state(STATE_FAILED, "set_model_state failed: %s" % exc)
            return False
        if not resp.success:
            self.set_state(STATE_FAILED, "set_model_state refused: %s" % resp.status_message)
            return False
        tol = float(cfg.get("tolerance_m", 0.03))
        t0 = time.time()
        pb = None
        while time.time() - t0 < 2.0 and not rospy.is_shutdown():
            pb = self.object_pose_base(name)
            if pb and self._near(pb, cfg["start_xyz_base"], tol):
                self.step_note = "%s reset to start %s (measured %s)" % (name, cfg["start_xyz_base"], [round(v, 3) for v in pb])
                rospy.loginfo("pendant_bridge: SCENE_RESET — %s", self.step_note)
                return True
            time.sleep(0.05)
        self.set_state(STATE_FAILED, "%s not at its start pose after reset (measured %s)" % (name, pb))
        return False

    def attach_object(self, name):
        """Kinematic attachment gripper link <-> object (fixed joint via gazebo_ros_link_attacher). Not a physics grasp."""
        cfg = self.object_cfg(name)
        if not cfg:
            self.set_state(STATE_FAILED, "unknown object '%s'" % name)
            return False
        if self.attach_srv is None:
            self.set_state(STATE_FAILED, "link attacher unavailable — install gazebo_ros_link_attacher and load its world plugin")
            return False
        pb = self.object_pose_base(name)
        tcp = self.current_tcp()
        if pb is None or tcp is None:
            self.set_state(STATE_FAILED, "attach: object pose or TCP pose unavailable")
            return False
        d = math.sqrt((tcp["x"] - pb[0]) ** 2 + (tcp["y"] - pb[1]) ** 2 + (tcp["z"] - pb[2]) ** 2)
        radius = float(cfg.get("attach_radius_m", 0.08))
        if d > radius:
            self.set_state(STATE_FAILED, "attach refused: TCP is %.3f m from %s (limit %.3f m) — not at the pick pose" % (d, name, radius))
            return False
        link = self.gripper_gazebo_link()
        req = AttachRequest()
        req.model_name_1, req.link_name_1 = self.robot_model, link
        req.model_name_2, req.link_name_2 = cfg.get("gazebo_model", name), cfg.get("gazebo_link", "link")
        try:
            resp = self.attach_srv(req)
        except Exception as exc:
            self.set_state(STATE_FAILED, "attach service failed: %s" % exc)
            return False
        if not resp.ok:
            self.set_state(STATE_FAILED, "attach refused by the link attacher (%s::%s <-> %s::%s)" % (
                self.robot_model, link, req.model_name_2, req.link_name_2))
            return False
        self.attached[name] = link
        self.step_note = "%s attached to %s::%s (TCP %.3f m from object centre) — kinematic joint, not a physics grasp" % (
            name, self.robot_model, link, d)
        rospy.loginfo("pendant_bridge: ATTACH — %s", self.step_note)
        return True

    def detach_object(self, name):
        cfg = self.object_cfg(name)
        if not cfg:
            self.set_state(STATE_FAILED, "unknown object '%s'" % name)
            return False
        if self.detach_srv is None:
            self.set_state(STATE_FAILED, "link attacher unavailable — cannot detach '%s'" % name)
            return False
        link = self.attached.get(name) or self.gripper_gazebo_link()
        req = AttachRequest()
        req.model_name_1, req.link_name_1 = self.robot_model, link
        req.model_name_2, req.link_name_2 = cfg.get("gazebo_model", name), cfg.get("gazebo_link", "link")
        try:
            resp = self.detach_srv(req)
        except Exception as exc:
            self.set_state(STATE_FAILED, "detach service failed: %s" % exc)
            return False
        if not resp.ok and name in self.attached:
            self.set_state(STATE_FAILED, "detach refused by the link attacher")
            return False
        self.attached.pop(name, None)
        time.sleep(0.5)                                  # let the object settle on its support
        self.step_note = "%s detached from %s::%s" % (name, self.robot_model, link)
        rospy.loginfo("pendant_bridge: DETACH — %s", self.step_note)
        return True

    def verify_object(self, name, expect, tol=None):
        """Measured check on the Gazebo object pose: lifted | at_place | at_start | attached."""
        cfg = self.object_cfg(name)
        if not cfg:
            self.set_state(STATE_FAILED, "unknown object '%s'" % name)
            return False
        if expect == "attached":
            ok = name in self.attached
            detail = "attached=%s" % ok
        else:
            pb = self.object_pose_base(name)
            if pb is None:
                self.set_state(STATE_FAILED, "verify_object: no fresh pose for '%s' (/gazebo/model_states)" % name)
                return False
            tol = float(tol if tol is not None else cfg.get("tolerance_m", 0.03))
            if expect == "lifted":
                min_lift = float(cfg.get("min_lift_m", 0.03))
                ok = pb[2] > float(cfg["start_xyz_base"][2]) + min_lift
                detail = "z %.3f vs start %.3f (+%.3f required)" % (pb[2], cfg["start_xyz_base"][2], min_lift)
            elif expect == "at_place":
                ok = self._near(pb, cfg["place_xyz_base"], tol)
                detail = "measured %s vs place target %s (tol %.3f)" % ([round(v, 3) for v in pb], cfg["place_xyz_base"], tol)
            elif expect == "at_start":
                ok = self._near(pb, cfg["start_xyz_base"], tol)
                detail = "measured %s vs start %s (tol %.3f)" % ([round(v, 3) for v in pb], cfg["start_xyz_base"], tol)
            else:
                self.set_state(STATE_FAILED, "verify_object: unknown expectation '%s'" % expect)
                return False
        if not ok:
            self.set_state(STATE_FAILED, "verify_object %s '%s' failed: %s" % (expect, name, detail))
            return False
        self.step_note = "%s %s: %s" % (name, expect, detail)
        rospy.loginfo("pendant_bridge: VERIFY — %s", self.step_note)
        return True

    def verify_tcp(self, pose_name, tol_m=0.01, tol_deg=3.0):
        """Measured check: current TCP (eef_tcp_link) vs the taught pose's stored TCP (position and orientation)."""
        key = str(pose_name).upper()
        rec = self.taught.get(key)
        if rec is None or not rec.get("tcp"):
            self.set_state(STATE_FAILED, "verify_tcp: '%s' is not a taught pose with a stored TCP" % key)
            return False
        cur = self.current_tcp()
        if cur is None:
            self.set_state(STATE_FAILED, "verify_tcp: current TCP pose unavailable")
            return False
        ref = rec["tcp"]
        dist = math.sqrt((cur["x"] - ref["x"]) ** 2 + (cur["y"] - ref["y"]) ** 2 + (cur["z"] - ref["z"]) ** 2)
        dot = abs(sum(a * b for a, b in zip(cur["q"], ref["q"])))
        ang = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
        detail = "TCP error %.1f mm / %.2f deg vs %s (tol %.0f mm / %.1f deg)" % (dist * 1000.0, ang, key, tol_m * 1000.0, tol_deg)
        if dist > tol_m or ang > tol_deg:
            self.set_state(STATE_FAILED, "verify_tcp failed: " + detail)
            return False
        self.step_note = detail
        rospy.loginfo("pendant_bridge: VERIFY_TCP — %s", detail)
        return True

    # ---- stop / estop ------------------------------------------------------
    def _hold_controllers(self):
        """Empty trajectory = cancel current trajectory and hold position (JointTrajectoryController)."""
        self.arm_cmd_pub.publish(JointTrajectory(joint_names=ARM_JOINTS))
        self.grip_cmd_pub.publish(JointTrajectory(joint_names=GRIP_JOINTS))

    def do_stop(self):
        self.stop_evt.set()
        self._drain_jobs()
        try:
            self.fjt.cancel_all_goals()
            self.group.stop()
        except Exception as exc:
            rospy.logwarn("pendant_bridge: stop: %s", exc)
        self._hold_controllers()
        if not self.task["running"] and self.state not in (STATE_EXECUTING, STATE_PLANNING):
            self.set_state(STATE_STOPPED)

    def do_estop(self):
        self.estop = True
        rospy.logwarn("pendant_bridge: SIMULATION E-STOP ACTIVATED (not a safety-rated stop)")
        self.do_stop()
        self.set_state(STATE_ESTOP, "simulation E-STOP active")

    def reset_estop(self):
        self.estop = False
        self.stop_evt.clear()
        with self.lock:
            self.last_error = ""
        self.set_state(STATE_IDLE)
        rospy.loginfo("pendant_bridge: E-STOP reset — system idle (no automatic task resume)")

    # ------------------------------------------------------- status/collision
    def _validity_now(self):
        """Fresh /check_state_validity call on the current state -> (valid | None if unavailable, contacts)."""
        if self.validity_srv is None:
            return None, []
        req = GetStateValidityRequest()
        req.robot_state = self.robot.get_current_state()
        req.group_name = PLANNING_GROUP
        resp = self.validity_srv(req)
        contacts = ["%s<->%s" % (c.contact_body_1, c.contact_body_2) for c in resp.contacts]
        return bool(resp.valid), contacts[:6]

    def check_collision(self, _evt):
        if self.validity_srv is None or self.state in (STATE_EXECUTING, STATE_PLANNING):
            return
        try:
            valid, contacts = self._validity_now()
            with self.lock:
                self.collision = {"valid": valid, "contacts": contacts}
        except Exception as exc:
            rospy.logwarn_throttle(10, "pendant_bridge: collision check failed: %s", exc)

    def publish_status(self, _evt):
        self.gripper_status()
        obj_status = self.object_status()          # computed outside the lock (object_pose_base locks itself)
        live = self.js.positions(ARM_JOINTS)
        with self.lock:
            target = [self.target_rad[n] for n in ARM_JOINTS] if self.target_rad else None
            if self.task["running"] and self.task_t0:
                self.task["cycle_time_s"] = round(time.time() - self.task_t0, 2)
            st = {
                "stamp": round(time.time(), 3),
                "bridge_ok": True,
                "state": self.state,
                "estop": self.estop,
                "estop_is_simulation_only": True,
                "speed": self.speed_pct,
                "sim": {"use_sim_time": self.use_sim_time, "rtf": self.rtf, "sim_time": round(rospy.get_time(), 2)},
                "last_cmd": dict(self.last_cmd),
                "last_error": self.last_error,
                "live_deg": [round(v, 2) for v in rad2deg(live)] if live else None,
                "vel_dps": [round(v, 1) for v in rad2deg(self.js.velocities(ARM_JOINTS))],
                "effort": [round(v, 2) for v in self.js.efforts(ARM_JOINTS)],
                "target_deg": [round(v, 2) for v in rad2deg(target)] if target else None,
                "err_deg": [round(math.degrees(l - t), 2) for l, t in zip(live, target)] if (live and target) else None,
                "tcp": None,
                "plan": dict(self.plan_info),
                "exec": dict(self.exec_info),
                "collision": dict(self.collision),
                "gripper": dict(self.gripper),
                "task": json.loads(json.dumps(self.task)),
                "teach_last": self.teach_last,
                "taught_count": len(self.taught),
                "object": obj_status,
                "task_history": list(self.history)[-10:],
                "scenario": self.scenario.get("name"),
                "workcell_loaded": list(self.workcell_loaded),
            }
        st["tcp"] = self.current_tcp()
        self.status_pub.publish(String(data=json.dumps(st)))


def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node("pendant_bridge")
    PendantBridge()
    rospy.spin()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()

