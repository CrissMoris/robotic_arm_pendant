#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
safety_tests.py — independent safety / motion test runner for the Mery6 simulation platform
(ROS Noetic + Gazebo + MoveIt + pendant_bridge).  SIMULATION ONLY.

Tests
  D1 normal_motion       (motion)   plan + execute a known safe taught pose, verify joint states reach it
  D2 unreachable_target  (planning) pose far outside the workspace is rejected, no trajectory is sent
  D3 joint_limit         (planning) out-of-limit joint targets are rejected by the bridge BEFORE planning
  D4 collision_check     (planning) a goal inside the table is invalid in the planning scene and cannot be planned;
                                    D4b (informative) planner routes around the coffee machine
  D5 estop_during_motion (motion)   simulation E-STOP during a controlled move; rejection after stop; resume after reset
  D6 barista_regression  (motion)   N consecutive Barista cycles, per-cycle planning/execution outcomes

Motion tests run only with --approve-motion AND an interactive "yes" per test.  Nothing here ever moves the robot on its own
after a failure: there is no automatic HOME / recovery motion.  The E-STOP exercised here is the bridge's simulation stop
(cancels goals, holds controllers, rejects commands); it is NOT a safety-rated industrial E-STOP.

Reports: orion5_interface/test_reports/safety_report_<YYYYmmdd_HHMMSS>_<pid>[_<n>].json + .csv (same stem, never overwritten).

Usage
  rosrun orion5_interface safety_tests.py --list
  rosrun orion5_interface safety_tests.py --tests D2,D3,D4
  rosrun orion5_interface safety_tests.py --tests D1,D5 --approve-motion
  rosrun orion5_interface safety_tests.py --tests D6 --approve-motion --barista-cycles 5 [--continue-on-failure]
  rosrun orion5_interface safety_tests.py --serve [--approve-motion]      # pendant panel mode (/pendant/test_cmd, /pendant/test_status)
Exit codes: 0 no FAIL (SKIP allowed), 1 at least one FAIL, 2 setup error.
"""

import argparse
import csv
import json
import math
import os
import platform
import socket
import sys
import threading
import time
import traceback
import types

import rospy
import rospkg
import moveit_commander

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest
import tf.transformations as tft

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pendant_bridge                      # noqa: E402  (validate_joint_limits / load_urdf_limits, main() is guarded)
from find_task_poses import PoseFinder, pose_from, quat_z_axis   # noqa: E402

ARM_JOINTS = pendant_bridge.ARM_JOINTS
PLANNING_GROUP = pendant_bridge.PLANNING_GROUP
EEF_LINK = pendant_bridge.EEF_LINK
REF_FRAME = pendant_bridge.REF_FRAME
CONTROLLER = pendant_bridge.ARM_CONTROLLER
FJT_GOAL_TOPIC = "/%s/follow_joint_trajectory/goal" % CONTROLLER
FJT_RESULT_TOPIC = "/%s/follow_joint_trajectory/result" % CONTROLLER
CMD_TOPIC = "/%s/command" % CONTROLLER

GOAL_STATUS_NAMES = {v: k for k, v in GoalStatus.__dict__.items() if isinstance(v, int) and k.isupper()}
MOVEIT_ERROR_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int) and k.isupper()}
TERMINAL_RESULTS = ("ok", "error", "rejected", "stopped")

TESTS = [
    ("D1", "normal_motion", True, "Plan and execute a move to a known safe taught pose; verify joint states reach it"),
    ("D2", "unreachable_target", False, "A pose far outside the workspace is rejected; no trajectory reaches the controller"),
    ("D3", "joint_limit", False, "Out-of-limit joint targets are rejected by the bridge before planning; boundary policy preserved"),
    ("D4", "collision_check", False, "A goal inside the table is invalid in the planning scene and cannot be planned (+ D4b route-around)"),
    ("D5", "estop_during_motion", True, "Simulation E-STOP during a controlled move: stop, reject, reset, resume"),
    ("D6", "barista_regression", True, "N consecutive Barista cycles with per-cycle planning/execution outcomes and the cup measured on the place pad"),
]
TEST_INFO = {t[0]: t for t in TESTS}
SIM_NOTE = "simulation only — the bridge E-STOP is not a safety-rated stop; no grasp is simulated"


def now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def deg2rad(v):
    return [math.radians(x) for x in v]


def rad2deg(v):
    return [math.degrees(x) for x in v]


# ============================================================ bridge status recorder
class StatusRecorder(object):
    """Subscribes to /pendant/status and /pendant/robot_info; keeps the latest dicts and a change log."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.latest_wall = 0.0
        self.info = None
        self.log = []          # (wall_t, state, last_cmd, plan, exec, task_step, estop)
        self._last_key = None
        rospy.Subscriber("/pendant/status", String, self._on_status, queue_size=5)
        rospy.Subscriber("/pendant/robot_info", String, self._on_info, queue_size=1)

    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.latest = st
            self.latest_wall = time.time()
            lc = st.get("last_cmd") or {}
            key = (st.get("state"), lc.get("id"), lc.get("result"), (st.get("task") or {}).get("step_index"), st.get("estop"),
                   (st.get("exec") or {}).get("goal_status"), (st.get("plan") or {}).get("ok"))
            if key != self._last_key:
                self._last_key = key
                self.log.append((time.time(), st.get("state"), dict(lc), dict(st.get("plan") or {}),
                                 dict(st.get("exec") or {}), (st.get("task") or {}).get("step_index"), st.get("estop")))

    def _on_info(self, msg):
        try:
            with self.lock:
                self.info = json.loads(msg.data)
        except ValueError:
            pass

    def get(self):
        with self.lock:
            return dict(self.latest) if self.latest else None

    def alive(self, max_age=1.5):
        with self.lock:
            return self.latest is not None and (time.time() - self.latest_wall) < max_age

    def wait_info(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            with self.lock:
                if self.info and self.latest:
                    return True
            time.sleep(0.1)
        return False

    def wait_result(self, cmd_id, timeout):
        """Wait until last_cmd.id == cmd_id with a terminal result. Returns (last_cmd, status) or (None, status)."""
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            st = self.get()
            if st:
                lc = st.get("last_cmd") or {}
                if lc.get("id") == cmd_id and lc.get("result") in TERMINAL_RESULTS:
                    return dict(lc), st
            time.sleep(0.05)
        return None, self.get()

    def wait_for(self, pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            st = self.get()
            if st and pred(st):
                return st
            time.sleep(0.05)
        return None

    def states_since(self, t_from):
        with self.lock:
            return [e[1] for e in self.log if e[0] >= t_from]

    def snapshots_for(self, cmd_id):
        """Latest plan/exec snapshots carrying this cmd_id (bridge tags them since Phase D-P1b)."""
        with self.lock:
            plan = exec_ = None
            for e in self.log:
                if e[3].get("cmd_id") == cmd_id:
                    plan = dict(e[3])
                if e[4].get("cmd_id") == cmd_id:
                    exec_ = dict(e[4])
            st = self.latest or {}
            if (st.get("plan") or {}).get("cmd_id") == cmd_id:
                plan = dict(st["plan"])
            if (st.get("exec") or {}).get("cmd_id") == cmd_id:
                exec_ = dict(st["exec"])
        return plan, exec_


# ============================================================ motion watch (proof of "no motion")
class MotionWatch(object):
    """Counts FollowJointTrajectory goals, /command messages and joint displacement during a window."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pos = None
        self.vel = [0.0] * 6
        self.base = None
        self.max_delta = 0.0
        self.max_vel = 0.0
        self.goals = 0
        self.results = []
        self.commands = 0
        self.active = False
        rospy.Subscriber("/joint_states", JointState, self._on_js, queue_size=1)
        rospy.Subscriber(FJT_GOAL_TOPIC, FollowJointTrajectoryActionGoal, self._on_goal, queue_size=10)
        rospy.Subscriber(FJT_RESULT_TOPIC, FollowJointTrajectoryActionResult, self._on_result, queue_size=10)
        rospy.Subscriber(CMD_TOPIC, JointTrajectory, self._on_cmd, queue_size=10)

    def _on_js(self, msg):
        names = list(msg.name)
        if not all(n in names for n in ARM_JOINTS):
            return
        pos = [msg.position[names.index(n)] for n in ARM_JOINTS]
        vel = [msg.velocity[names.index(n)] if len(msg.velocity) == len(names) else 0.0 for n in ARM_JOINTS]
        with self.lock:
            self.pos, self.vel = pos, vel
            if self.active and self.base is not None:
                self.max_delta = max(self.max_delta, max(abs(p - b) for p, b in zip(pos, self.base)))
                self.max_vel = max(self.max_vel, max(abs(v) for v in vel))

    def _on_goal(self, _msg):
        with self.lock:
            if self.active:
                self.goals += 1

    def _on_result(self, msg):
        with self.lock:
            if self.active:
                self.results.append(GOAL_STATUS_NAMES.get(msg.status.status, str(msg.status.status)))

    def _on_cmd(self, _msg):
        with self.lock:
            if self.active:
                self.commands += 1

    def current(self):
        with self.lock:
            return (list(self.pos) if self.pos else None), list(self.vel)

    def wait_joint_states(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            if self.pos is not None:
                return True
            time.sleep(0.05)
        return False

    def start(self):
        with self.lock:
            self.base = list(self.pos) if self.pos else None
            self.max_delta = self.max_vel = 0.0
            self.goals = self.commands = 0
            self.results = []
            self.active = True

    def rebase(self):
        with self.lock:
            self.base = list(self.pos) if self.pos else None
            self.max_delta = 0.0

    def snapshot(self, stop=True):
        with self.lock:
            snap = {"fjt_goals": self.goals, "command_msgs": self.commands, "fjt_results": list(self.results),
                    "max_joint_delta_deg": round(math.degrees(self.max_delta), 3),
                    "max_vel_dps": round(math.degrees(self.max_vel), 2)}
            if stop:
                self.active = False
        return snap


# ============================================================ bridge client
class BridgeClient(object):
    def __init__(self, recorder):
        self.rec = recorder
        self.pub = rospy.Publisher("/pendant/cmd", String, queue_size=10)
        self.seq = 0
        self.prefix = "t%d_" % os.getpid()
        rospy.sleep(0.5)

    def send(self, cmd):
        self.seq += 1
        cmd = dict(cmd)
        cmd["id"] = self.prefix + str(self.seq)
        self.pub.publish(String(data=json.dumps(cmd)))
        return cmd["id"]

    def call(self, cmd, timeout):
        """send + wait for the terminal result. Returns (cmd_id, last_cmd|None, status, plan, exec)."""
        cid = self.send(cmd)
        lc, st = self.rec.wait_result(cid, timeout)
        plan, ex = self.rec.snapshots_for(cid)
        return cid, lc, st, plan, ex

    def resolve_pose(self, name):
        cid, lc, _st, _p, _e = self.call({"cmd": "resolve_pose", "name": name}, 5.0)
        if lc and lc.get("result") == "ok":
            try:
                return json.loads(lc.get("message") or "{}")
            except ValueError:
                return None
        return None


# ============================================================ MoveIt probe (planning only)
class Probe(PoseFinder):
    """PoseFinder + raw IK (collision-aware or not), FK, planning-only plan with error code. Never executes."""

    def __init__(self, cfg, ready_deg):
        PoseFinder.__init__(self, cfg, ready_deg)
        self.fk_srv = None
        try:
            rospy.wait_for_service("/compute_fk", timeout=5.0)
            self.fk_srv = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        except rospy.ROSException:
            pass

    def ik_raw(self, pose, seed_rad, avoid_collisions, attempts=3, timeout=0.5):
        req = GetPositionIKRequest()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.ik_link_name = EEF_LINK
        req.ik_request.robot_state = self.state_with(seed_rad)
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = REF_FRAME
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout = rospy.Duration(timeout)
        last_code = None
        for _ in range(attempts):
            resp = self.ik_srv(req)
            last_code = MOVEIT_ERROR_NAMES.get(resp.error_code.val, str(resp.error_code.val))
            if resp.error_code.val == MoveItErrorCodes.SUCCESS:
                js = resp.solution.joint_state
                sol = [js.position[list(js.name).index(n)] for n in ARM_JOINTS]
                if self.within_limits(sol):
                    return sol, last_code
        return None, last_code

    def fk(self, joints_rad):
        if self.fk_srv is None:
            return None
        req = GetPositionFKRequest()
        req.header.frame_id = REF_FRAME
        req.fk_link_names = [EEF_LINK]
        req.robot_state = self.state_with(joints_rad)
        resp = self.fk_srv(req)
        if resp.error_code.val != MoveItErrorCodes.SUCCESS or not resp.pose_stamped:
            return None
        p = resp.pose_stamped[0].pose.position
        return [p.x, p.y, p.z]

    def plan_only(self, goal_rad, start_rad=None):
        """OMPL plan to a joint goal (nothing executed). Returns (ok, error_code_name, planning_time_s, exception)."""
        try:
            if start_rad is not None:
                self.group.set_start_state(self.state_with(start_rad))
            else:
                self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(goal_rad)
            t0 = time.time()
            res = self.group.plan()
            dt = time.time() - t0
        except Exception as exc:
            try:
                self.group.clear_pose_targets()
                self.group.set_start_state_to_current_state()
            except Exception:
                pass
            return False, None, None, "%s: %s" % (type(exc).__name__, exc)
        finally:
            try:
                self.group.clear_pose_targets()
                self.group.set_start_state_to_current_state()
            except Exception:
                pass
        if isinstance(res, tuple):
            ok, traj, _t, err = res
            code = err.val if hasattr(err, "val") else int(err)
            ok = bool(ok) and len(traj.joint_trajectory.points) > 0
        else:
            ok = len(res.joint_trajectory.points) > 0
            code = MoveItErrorCodes.SUCCESS if ok else MoveItErrorCodes.PLANNING_FAILED
        return ok, MOVEIT_ERROR_NAMES.get(code, str(code)), round(dt, 3), None

    def interp_invalid(self, start_rad, goal_rad, n=50):
        """First invalid state on the straight joint-space interpolation (validity + contacts), or None."""
        for i in range(1, n):
            a = i / float(n)
            q = [s + a * (g - s) for s, g in zip(start_rad, goal_rad)]
            valid, contacts = self.validity(q)
            if not valid:
                return {"fraction": round(a, 3), "contacts": contacts}
        return None


# ============================================================ report
class Report(object):
    def __init__(self, out_dir, argv, approve_motion):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = "safety_report_%s_%d" % (stamp, os.getpid())
        self.stem = None
        for n in range(0, 1000):
            cand = base if n == 0 else "%s_%d" % (base, n)
            path = os.path.join(out_dir, cand + ".json")
            try:
                fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
                os.close(fd)
                self.stem = cand
                break
            except FileExistsError:
                continue
        if self.stem is None:
            raise RuntimeError("could not reserve a unique report name in %s" % out_dir)
        self.json_path = os.path.join(out_dir, self.stem + ".json")
        self.csv_path = os.path.join(out_dir, self.stem + ".csv")
        self.data = {"schema_version": 1, "run_id": self.stem, "generated_at": now_iso(), "simulation_only": True,
                     "environment": {"runner": {"script": "safety_tests.py", "argv": argv, "approve_motion": approve_motion}},
                     "summary": {}, "tests": [], "barista": None}
        self.t0 = time.time()

    def add(self, rec):
        self.data["tests"].append(rec)
        self.write()

    def write(self):
        tests = self.data["tests"]
        failed = [{"test_id": t["test_id"], "failure_step": t.get("failure_step"), "error_message": t.get("error_message")}
                  for t in tests if t["result"] == "FAIL"]
        self.data["summary"] = {"total": len(tests),
                                "pass": sum(1 for t in tests if t["result"] == "PASS"),
                                "fail": len(failed),
                                "skip": sum(1 for t in tests if t["result"] == "SKIP"),
                                "duration_sec": round(time.time() - self.t0, 2), "failed": failed}
        with open(self.json_path, "w") as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False, default=str)
        cols = ["test_id", "name", "result", "start_time", "duration_sec", "plan_success", "execution_success",
                "expected_behavior", "actual_behavior", "failure_step", "error_message", "collision_contacts",
                "moveit_error_code", "fjt_goals", "command_msgs", "max_joint_delta_deg", "report_json"]
        with open(self.csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for t in tests:
                ci = t.get("collision_info") or {}
                mo = t.get("motion_observed") or {}
                w.writerow([t.get("test_id"), t.get("name"), t.get("result"), t.get("start_time"), t.get("duration_sec"),
                            t.get("plan_success"), t.get("execution_success"), t.get("expected_behavior"),
                            t.get("actual_behavior"), t.get("failure_step"), t.get("error_message"),
                            "; ".join(ci.get("contacts") or []) if isinstance(ci, dict) else "",
                            ci.get("moveit_error_code") if isinstance(ci, dict) else "",
                            mo.get("fjt_goals"), mo.get("command_msgs"), mo.get("max_joint_delta_deg"), self.json_path])
                for k, c in enumerate(t.get("cycles") or []):
                    w.writerow(["%s.cycle%d" % (t["test_id"], k + 1), t.get("name"), "PASS" if c.get("result") == "ok" else "FAIL",
                                c.get("start_time"), c.get("duration_sec"), c.get("plan_success"), c.get("execution_success"),
                                "task cycle completes with result ok", c.get("result"), c.get("failure_step"), c.get("error_message"),
                                "", "", "", "", "", self.json_path])

    def print_summary(self):
        s = self.data["summary"]
        print("\n=== safety tests summary ===")
        for t in self.data["tests"]:
            print("  %-4s %-20s %-5s %6.1fs  %s" % (t["test_id"], t["name"], t["result"], t.get("duration_sec") or 0.0,
                                                    (t.get("actual_behavior") or "")[:110]))
        print("  total %d  PASS %d  FAIL %d  SKIP %d" % (s["total"], s["pass"], s["fail"], s["skip"]))
        for f in s["failed"]:
            print("  FAILED %s: %s%s" % (f["test_id"], f.get("error_message"), (" (step %s)" % f["failure_step"]) if f.get("failure_step") else ""))
        print("  report: %s\n          %s" % (self.json_path, self.csv_path))


def new_record(test_id):
    tid, name, motion, expected = TEST_INFO[test_id]
    return {"test_id": tid, "name": name, "motion_test": motion, "result": "SKIP", "start_time": now_iso(),
            "duration_sec": None, "plan_success": None, "execution_success": None,
            "stages": {"plan": "not_attempted", "execution": "not_attempted"},
            "expected_behavior": expected, "actual_behavior": None, "failure_step": None, "error_message": None,
            "collision_info": None, "motion_observed": None, "bridge": {"cmd_ids": [], "state_sequence": [], "plan": None, "exec": None},
            "substeps": [], "cycles": None, "notes": [SIM_NOTE]}


def finish(rec, t0, result, actual, error=None, failure_step=None):
    rec["result"] = result
    rec["actual_behavior"] = actual
    rec["error_message"] = error
    rec["failure_step"] = failure_step
    rec["duration_sec"] = round(time.time() - t0, 2)
    return rec


def substep(rec, name, ok, detail):
    rec["substeps"].append({"name": name, "ok": ok, "detail": detail})
    return ok


# ============================================================ context + helpers
class Ctx(object):
    def __init__(self, args):
        self.args = args
        self.rec = StatusRecorder()
        self.watch = MotionWatch()
        self.bridge = BridgeClient(self.rec)
        self.probe = None
        self.params = {k: rospy.get_param("/pendant_bridge/" + k, None)
                       for k in ("joint_tol_rad", "vel_tol_rad_s", "settle_time", "cartesian_min_fraction", "default_speed_pct")}
        self.joint_tol = float(self.params.get("joint_tol_rad") or 0.02)
        self.vel_tol = float(self.params.get("vel_tol_rad_s") or 0.02)
        self.no_motion_tol = math.radians(args.no_motion_tol_deg)
        self.limits = pendant_bridge.load_urdf_limits()
        self.workcell = rospy.get_param("/pendant_bridge/workcell", []) or []
        self.initial_speed = None
        self.abort = False

    def info(self):
        return self.rec.info or {}

    def status(self):
        return self.rec.get() or {}

    def taught(self, name):
        tp = (self.info().get("taught_poses") or {}).get(name)
        return tp

    def resolved_deg(self, name):
        r = self.bridge.resolve_pose(name)
        return r.get("joints_deg") if r else None

    def at(self, joints_deg, tol=None):
        pos, _ = self.watch.current()
        if pos is None or joints_deg is None:
            return False
        tol = self.joint_tol if tol is None else tol
        return max(abs(p - g) for p, g in zip(pos, deg2rad(joints_deg))) < tol

    def preflight(self, rec, need_scene=False):
        st = self.status()
        if not self.rec.alive():
            return substep(rec, "preflight", False, "pendant_bridge status not alive")
        if st.get("estop"):
            return substep(rec, "preflight", False, "E-STOP is active — reset it first")
        if st.get("state") in ("PLANNING", "EXECUTING") or (st.get("task") or {}).get("running"):
            return substep(rec, "preflight", False, "bridge busy (state %s)" % st.get("state"))
        pos, _ = self.watch.current()
        if pos is None:
            return substep(rec, "preflight", False, "no /joint_states")
        if self.probe is not None:
            valid, contacts = self.probe.validity(pos)
            if not valid:
                return substep(rec, "preflight", False, "current state in collision: %s" % contacts)
        if need_scene and "table" not in (self.info().get("workcell_loaded") or []):
            return substep(rec, "preflight", False, "planning scene incomplete: 'table' not loaded")
        return substep(rec, "preflight", True, "bridge %s, estop false, state valid" % st.get("state"))

    def approve(self, rec, test_id):
        if not self.args.approve_motion:
            rec["notes"].append("motion not approved (--approve-motion + interactive yes)")
            return False
        if self.args.serve:
            return bool(getattr(self.args, "_serve_approved", False))
        try:
            ans = input('Type "yes" to start %s (%s) — the robot WILL MOVE in Gazebo (simulation only): ' % (test_id, TEST_INFO[test_id][1]))
        except EOFError:
            ans = ""
        if ans.strip().lower() != "yes":
            rec["notes"].append("motion not approved at the prompt")
            return False
        return True

    def bridge_motion(self, rec, cmd, timeout):
        """Send a motion command through the bridge and record plan/exec/state. Returns (ok, lc, plan, exec, st)."""
        t0 = time.time()
        cid, lc, st, plan, ex = self.bridge.call(cmd, timeout)
        rec["bridge"]["cmd_ids"].append(cid)
        rec["bridge"]["state_sequence"] = self.rec.states_since(t0)[-20:]
        rec["bridge"]["plan"], rec["bridge"]["exec"] = plan, ex
        ok = bool(lc) and lc.get("result") == "ok"
        return ok, lc, plan, ex, st

    def no_motion(self, snap):
        return snap["fjt_goals"] == 0 and snap["command_msgs"] == 0 and math.radians(snap["max_joint_delta_deg"]) < self.no_motion_tol


def tool_down_q(ctx):
    rd = ctx.taught("READY")
    q = (rd or {}).get("tcp", {}).get("q") if rd else None
    if q and quat_z_axis(q)[2] < -0.9:
        return [float(v) for v in q]
    return [float(v) for v in tft.quaternion_from_euler(math.pi, 0.0, 0.0)]


# ============================================================ tests
def test_d1(ctx):
    rec = new_record("D1")
    t0 = time.time()
    if not ctx.preflight(rec):
        return finish(rec, t0, "SKIP", "preflight failed", rec["substeps"][-1]["detail"])
    # target: first taught candidate the arm is NOT already at
    target = None
    for name in ("READY", "PRE_PICK"):
        if ctx.taught(name) and not ctx.at(ctx.taught(name)["joints_deg"]):
            target = name
            break
    if target is None:
        return finish(rec, t0, "SKIP", "no suitable taught target (READY/PRE_PICK missing or arm already there)")
    if not ctx.approve(rec, "D1"):
        return finish(rec, t0, "SKIP", "motion test not approved", "motion not approved")
    goal_deg = ctx.taught(target)["joints_deg"]
    ctx.watch.start()
    ok, lc, plan, ex, st = ctx.bridge_motion(rec, {"cmd": "move_named", "name": target, "speed": ctx.args.speed}, 90.0)
    snap = ctx.watch.snapshot()
    rec["motion_observed"] = snap
    plan_ok = bool(plan and plan.get("attempted") and plan.get("ok"))
    rec["stages"]["plan"] = "attempted" if plan and plan.get("attempted") else "not_attempted"
    rec["plan_success"] = plan_ok if plan and plan.get("attempted") else None
    exec_attempted = bool(ex and ex.get("attempted"))
    rec["stages"]["execution"] = "attempted" if exec_attempted else "not_attempted"
    reached = ctx.at(goal_deg)
    exec_ok = exec_attempted and ex.get("goal_status") == "SUCCEEDED" and (st or {}).get("state") == "TARGET_REACHED" and reached
    rec["execution_success"] = exec_ok if exec_attempted else None
    substep(rec, "plan", plan_ok, "plan %s" % json.dumps(plan))
    substep(rec, "execute", exec_ok, "exec %s, state %s, joints within %.3f rad: %s" % (
        json.dumps(ex), (st or {}).get("state"), ctx.joint_tol, reached))
    actual = "target %s: plan_ok=%s, goal_status=%s, state=%s, reached=%s, fjt_goals=%d" % (
        target, plan_ok, (ex or {}).get("goal_status"), (st or {}).get("state"), reached, snap["fjt_goals"])
    if ok and plan_ok and exec_ok:
        return finish(rec, t0, "PASS", actual)
    step = "plan" if not plan_ok else "execute"
    return finish(rec, t0, "FAIL", actual, (lc or {}).get("message") or (st or {}).get("last_error") or "see substeps", step)


def test_d2(ctx):
    rec = new_record("D2")
    t0 = time.time()
    if not ctx.preflight(rec):
        return finish(rec, t0, "SKIP", "preflight failed", rec["substeps"][-1]["detail"])
    xyz = [1.5, 0.0, 0.5]
    q = tool_down_q(ctx)
    # direct IK (no collision avoidance) — expected: no solution
    pos, _ = ctx.watch.current()
    sol, code = ctx.probe.ik_raw(pose_from(xyz, q), pos, avoid_collisions=False)
    ik_rejected = sol is None
    substep(rec, "compute_ik", ik_rejected, "error_code %s" % code)
    rec["collision_info"] = {"moveit_error_code": code, "target_xyz": xyz}
    # bridge move_pose — expected: FAILED, nothing sent
    rpy = [round(math.degrees(v), 2) for v in tft.euler_from_quaternion(q)]
    ctx.watch.start()
    ok, lc, plan, ex, st = ctx.bridge_motion(rec, {"cmd": "move_pose", "xyz": xyz, "rpy_deg": rpy}, 40.0)
    snap = ctx.watch.snapshot()
    rec["motion_observed"] = snap
    rejected = bool(lc) and lc.get("result") == "error"
    rec["stages"]["plan"] = "attempted" if plan and plan.get("attempted") else "not_attempted"
    rec["plan_success"] = (False if plan and plan.get("attempted") else None)
    rec["stages"]["execution"] = "not_applicable"
    nomo = ctx.no_motion(snap)
    substep(rec, "bridge_rejects", rejected, "result %s: %s (plan %s)" % ((lc or {}).get("result"), (lc or {}).get("message"), json.dumps(plan)))
    substep(rec, "no_motion", nomo, json.dumps(snap))
    actual = "compute_ik %s; bridge result %s (%s); fjt_goals %d, commands %d, delta %.2f deg" % (
        code, (lc or {}).get("result"), (lc or {}).get("message"), snap["fjt_goals"], snap["command_msgs"], snap["max_joint_delta_deg"])
    if ik_rejected and rejected and nomo:
        return finish(rec, t0, "PASS", actual)
    return finish(rec, t0, "FAIL", actual, "unreachable target was not rejected or motion observed",
                  "compute_ik" if not ik_rejected else ("bridge" if not rejected else "no_motion"))


def test_d3(ctx):
    rec = new_record("D3")
    t0 = time.time()
    if not ctx.preflight(rec):
        return finish(rec, t0, "SKIP", "preflight failed", rec["substeps"][-1]["detail"])
    if not ctx.limits:
        return finish(rec, t0, "SKIP", "URDF limits not available (urdf_parser_py)")
    # limits from the URDF (same source as the bridge) cross-checked with robot_info
    info_j = {j["name"]: j for j in (ctx.info().get("joints") or [])}
    mismatch = [n for n in ARM_JOINTS if n in info_j and abs(info_j[n]["max_deg"] - math.degrees(ctx.limits[n]["upper"])) > 0.05]
    substep(rec, "limits_consistent", not mismatch, "robot_info vs URDF upper limits mismatch: %s" % mismatch)
    pos, _ = ctx.watch.current()
    cur_deg = rad2deg(pos)
    all_ok = True
    details = []
    ctx.watch.start()
    for i, n in enumerate(ARM_JOINTS):
        target = list(cur_deg)
        target[i] = math.degrees(ctx.limits[n]["upper"]) + 5.0
        ok, lc, plan, ex, st = ctx.bridge_motion(rec, {"cmd": "move_joints", "deg": [round(v, 2) for v in target]}, 15.0)
        rejected = bool(lc) and lc.get("result") == "error" and "joint limit" in (lc.get("message") or "")
        not_planned = bool(plan) and plan.get("cmd_id") == rec["bridge"]["cmd_ids"][-1] and not plan.get("attempted")
        not_executed = not (ex and ex.get("attempted"))
        case_ok = rejected and not_planned and not_executed
        all_ok = all_ok and case_ok
        details.append("%s: %s / planned=%s" % (n, (lc or {}).get("message"), not not_planned))
        substep(rec, "reject_%s" % n, case_ok, "result %s, plan %s, exec %s" % (json.dumps(lc), json.dumps(plan), json.dumps(ex)))
    snap = ctx.watch.snapshot()
    rec["motion_observed"] = snap
    nomo = ctx.no_motion(snap)
    substep(rec, "no_motion", nomo, json.dumps(snap))
    # boundary policy through the bridge's own validator (no duplicated limits, no motion)
    ns = types.SimpleNamespace(limits=ctx.limits)
    vj = pendant_bridge.PendantBridge.validate_joint_limits
    up = ctx.limits["joint_3"]["upper"]
    base = [0.0] * 6
    def chk(v):
        t = list(base); t[2] = v
        return vj(ns, t)
    b_exact, b_slack, b_over = chk(up), chk(up + 1e-7), chk(up + 1e-5)
    boundary_ok = b_exact is None and b_slack is None and b_over is not None
    substep(rec, "boundary_policy", boundary_ok, "exact=%s, +1e-7=%s, +1e-5=%s" % (b_exact, b_slack, b_over))
    # MoveIt behaviour: recorded only
    try:
        over = list(pos); over[2] = up + 0.1
        ctx.probe.group.set_joint_value_target(over)
        ctx.probe.group.clear_pose_targets()
        moveit_note = "set_joint_value_target(upper+0.1 rad) accepted silently (no exception)"
    except Exception as exc:
        moveit_note = "set_joint_value_target(upper+0.1 rad) raised %s: %s" % (type(exc).__name__, str(exc)[:120])
    rec["notes"].append("MoveIt behaviour (recorded, not asserted): " + moveit_note)
    rec["stages"]["plan"] = "not_attempted"
    rec["stages"]["execution"] = "not_applicable"
    actual = "6/6 rejected before planning: %s; no motion: %s; boundary policy ok: %s" % (all_ok, nomo, boundary_ok)
    if all_ok and nomo and boundary_ok and not mismatch:
        return finish(rec, t0, "PASS", actual)
    return finish(rec, t0, "FAIL", actual, "; ".join(details)[:300],
                  "reject" if not all_ok else ("no_motion" if not nomo else "boundary_policy"))


def table_box(ctx):
    for o in ctx.workcell:
        if o.get("name") == "table":
            return o
    return None


def test_d4(ctx):
    rec = new_record("D4")
    t0 = time.time()
    if not ctx.preflight(rec, need_scene=True):
        return finish(rec, t0, "SKIP", "preflight failed", rec["substeps"][-1]["detail"])
    tb = table_box(ctx)
    if not tb:
        return finish(rec, t0, "SKIP", "table not found in /pendant_bridge/workcell")
    pos, _ = ctx.watch.current()
    pick = ctx.taught("PICK")
    if pick:
        tcp = pick["tcp"]
        xyz, q, seed = [tcp["x"], tcp["y"], 0.0], [float(v) for v in tcp["q"]], deg2rad(pick["joints_deg"])
    else:
        xyz, q, seed = [0.42, -0.15, 0.0], tool_down_q(ctx), (deg2rad(ctx.taught("READY")["joints_deg"]) if ctx.taught("READY") else pos)
    sol, code = ctx.probe.ik_raw(pose_from(xyz, q), seed, avoid_collisions=False)
    if sol is None:
        rec["collision_info"] = {"goal_tcp": xyz, "moveit_error_code": code}
        return finish(rec, t0, "SKIP", "no colliding IK solution found for TCP %s (code %s) — scenario could not be constructed" % (xyz, code))
    valid, contacts = ctx.probe.validity(sol)
    fk = ctx.probe.fk(sol)
    cx, cy, cz = tb["xyz"]
    sx, sy, sz = tb["size"]
    inside = bool(fk) and abs(fk[0] - cx) <= sx / 2 and abs(fk[1] - cy) <= sy / 2 and abs(fk[2] - cz) <= sz / 2
    table_contact = any("table" in c for c in contacts)
    rec["collision_info"] = {"goal_valid": valid, "contacts": contacts, "goal_joints_deg": [round(v, 2) for v in rad2deg(sol)],
                             "goal_tcp": xyz, "fk_tcp": fk, "tcp_inside_table_box": inside, "moveit_error_code": None}
    substep(rec, "goal_invalid_with_table_contact", (not valid) and table_contact, "valid=%s contacts=%s" % (valid, contacts))
    substep(rec, "geometric_cross_check", inside, "fk %s inside table box centre %s size %s: %s" % (fk, tb["xyz"], tb["size"], inside))
    if valid:
        return finish(rec, t0, "FAIL", "goal expected in collision but the planning scene reports it valid", "collision not detected", "validity")
    if not table_contact:
        return finish(rec, t0, "FAIL", "state invalid but no contact names the table: %s" % contacts, "collision reported without the intended object", "validity")
    if not inside:
        return finish(rec, t0, "FAIL", "goal TCP %s is not inside the table box" % fk, "geometric cross-check failed", "geometry")
    ctx.watch.start()
    ok, ecode, dt, exc = ctx.probe.plan_only(sol)
    rec["collision_info"]["moveit_error_code"] = ecode if exc is None else exc
    rec["stages"]["plan"] = "attempted"
    rec["plan_success"] = ok
    rec["stages"]["execution"] = "not_applicable"
    substep(rec, "plan_rejected", not ok, "plan ok=%s code=%s time=%s exc=%s" % (ok, ecode, dt, exc))
    via = None
    if ctx.args.via_bridge:
        if ctx.approve(rec, "D4"):
            ok_b, lc, plan, ex, st = ctx.bridge_motion(rec, {"cmd": "move_joints", "deg": [round(v, 2) for v in rad2deg(sol)]}, 40.0)
            via = (lc or {}).get("result") == "error" and not (ex and ex.get("attempted"))
            substep(rec, "bridge_rejects", bool(via), "result %s: %s plan %s" % ((lc or {}).get("result"), (lc or {}).get("message"), json.dumps(plan)))
        else:
            rec["notes"].append("--via-bridge requested but motion not approved; skipped")
    snap = ctx.watch.snapshot()
    rec["motion_observed"] = snap
    nomo = ctx.no_motion(snap)
    substep(rec, "no_motion", nomo, json.dumps(snap))
    # D4b informative: route around the coffee machine (planning only)
    d4b = "not run"
    try:
        q2 = tool_down_q(ctx)
        s_sol = ctx.probe.ik(pose_from([0.62, -0.32, 0.12], q2), pos)
        g_sol = ctx.probe.ik(pose_from([0.62, 0.32, 0.12], q2), pos)
        if s_sol is None or g_sol is None:
            d4b = "SKIP: endpoints not reachable collision-free"
        else:
            blocked = ctx.probe.interp_invalid(s_sol, g_sol)
            if not blocked or not any("coffee_machine" in c for c in blocked["contacts"]):
                d4b = "SKIP: straight joint interpolation not blocked by coffee_machine (%s)" % blocked
            else:
                ok2, code2, dt2, exc2 = ctx.probe.plan_only(g_sol, start_rad=s_sol)
                d4b = ("PASS: collision-free alternative path found (%.2fs)" % dt2) if ok2 else ("FAIL: no path around obstacle (%s %s)" % (code2, exc2))
    except Exception as exc:
        d4b = "error: %s" % exc
    rec["notes"].append("D4b route-around (informative): " + d4b)
    substep(rec, "d4b_route_around", not d4b.startswith("FAIL"), d4b)
    actual = "goal invalid (%s); plan ok=%s code=%s; no motion=%s; D4b %s" % (", ".join(contacts[:3]), ok, ecode, nomo, d4b[:60])
    if (not ok) and nomo and (via is None or via):
        return finish(rec, t0, "PASS", actual)
    if ok:
        return finish(rec, t0, "FAIL", actual, "collision not enforced: planner returned a plan to an invalid goal", "plan")
    return finish(rec, t0, "FAIL", actual, "motion observed or bridge did not reject", "no_motion" if not nomo else "bridge")


def test_d5(ctx):
    rec = new_record("D5")
    t0 = time.time()
    if not ctx.preflight(rec):
        return finish(rec, t0, "SKIP", "preflight failed", rec["substeps"][-1]["detail"])
    ready = ctx.taught("READY")
    if not ready:
        return finish(rec, t0, "SKIP", "READY not taught")
    home = ctx.bridge.resolve_pose("HOME")
    if not home:
        return finish(rec, t0, "SKIP", "resolve_pose HOME failed (bridge without resolve_pose?)")
    home_deg = home["joints_deg"]
    err = pendant_bridge.PendantBridge.validate_joint_limits(types.SimpleNamespace(limits=ctx.limits), deg2rad(home_deg))
    valid, contacts = ctx.probe.validity(deg2rad(home_deg))
    okp, code, dt, exc = ctx.probe.plan_only(deg2rad(home_deg), start_rad=deg2rad(ready["joints_deg"]))
    substep(rec, "target_prevalidation", err is None and valid and okp, "HOME via %s = %s; limits %s; valid %s %s; plan READY->HOME %s (%s)" % (
        home.get("source"), home_deg, err or "ok", valid, contacts, okp, code or exc))
    if not (err is None and valid and okp):
        return finish(rec, t0, "SKIP", "motion target failed pre-validation; nothing moved")
    if ctx.at(home_deg) and ctx.at(ready["joints_deg"]):
        return finish(rec, t0, "SKIP", "READY and HOME coincide with the current pose — no measurable move for the E-STOP test")
    if not ctx.approve(rec, "D5"):
        return finish(rec, t0, "SKIP", "motion test not approved", "motion not approved")
    # 1. ensure READY (approved motion)
    if not ctx.at(ready["joints_deg"]):
        ok, lc, plan, ex, st = ctx.bridge_motion(rec, {"cmd": "move_named", "name": "READY", "speed": ctx.args.speed}, 90.0)
        if not substep(rec, "go_ready", ok, "result %s" % json.dumps(lc)):
            return finish(rec, t0, "FAIL", "could not reach READY before the test", (lc or {}).get("message"), "go_ready")
    # 2. start a slow move to HOME and wait until it is really executing
    ctx.watch.start()
    move_id = ctx.bridge.send({"cmd": "move_named", "name": "HOME", "speed": 10})
    rec["bridge"]["cmd_ids"].append(move_id)
    moving = ctx.rec.wait_for(lambda s: s.get("state") == "EXECUTING", 20.0)
    if moving:
        t_m = time.time()
        while time.time() - t_m < 10.0:
            _, vel = ctx.watch.current()
            if max(abs(v) for v in vel) > 2 * ctx.vel_tol:
                break
            time.sleep(0.05)
        else:
            moving = None
    if not substep(rec, "motion_in_progress", bool(moving), "state EXECUTING with joint velocity observed"):
        ctx.bridge.send({"cmd": "stop"})
        return finish(rec, t0, "FAIL", "move never reached EXECUTING with velocity", "no motion to interrupt", "motion_in_progress")
    time.sleep(1.0)
    # 3. E-STOP
    t_es = time.time()
    ctx.bridge.send({"cmd": "estop", "active": True})
    es = ctx.rec.wait_for(lambda s: s.get("estop") and s.get("state") == "ESTOP", 1.0)
    t_state = round(time.time() - t_es, 2)
    t_v = time.time()
    stopped = False
    while time.time() - t_v < 1.5:
        _, vel = ctx.watch.current()
        if max(abs(v) for v in vel) < ctx.vel_tol:
            stopped = True
            break
        time.sleep(0.02)
    t_stop = round(time.time() - t_es, 2)
    ctx.watch.rebase()
    time.sleep(1.0)
    pos_a, _ = ctx.watch.current()
    hold = ctx.watch.snapshot(stop=False)["max_joint_delta_deg"] < math.degrees(ctx.joint_tol)
    stx = ctx.status()
    preempted = (stx.get("exec") or {}).get("goal_status") == "PREEMPTED"
    substep(rec, "estop_state", bool(es), "state=%s estop=%s after %.2fs" % (stx.get("state"), stx.get("estop"), t_state))
    substep(rec, "goal_preempted", preempted, "exec %s" % json.dumps(stx.get("exec")))
    substep(rec, "joints_stopped", stopped, "velocities < %.3f rad/s after %.2fs" % (ctx.vel_tol, t_stop))
    substep(rec, "position_hold", hold, "max delta over 1 s = %.3f deg" % ctx.watch.snapshot(stop=False)["max_joint_delta_deg"])
    # 4. new motion must be rejected
    goals_before = ctx.watch.snapshot(stop=False)["fjt_goals"]
    cid, lc, st, plan, ex = ctx.bridge.call({"cmd": "move_named", "name": "READY"}, 5.0)
    rec["bridge"]["cmd_ids"].append(cid)
    rej = bool(lc) and lc.get("result") == "rejected" and "E-STOP" in (lc.get("message") or "")
    goals_after = ctx.watch.snapshot(stop=False)["fjt_goals"]
    substep(rec, "rejected_while_estop", rej and goals_after == goals_before, "result %s; fjt goals %d->%d" % (json.dumps(lc), goals_before, goals_after))
    # 5. reset (only after the interrupted job reported 'stopped'); an early reset must be rejected with 'retry'
    ctx.rec.wait_for(lambda s: (s.get("last_cmd") or {}).get("id") == move_id and (s.get("last_cmd") or {}).get("result") in TERMINAL_RESULTS, 10.0)
    reset_ok = False
    retries = 0
    for _ in range(10):
        cid, lc, st, _p, _e = ctx.bridge.call({"cmd": "estop", "active": False}, 3.0)
        if lc and lc.get("result") == "ok":
            reset_ok = True
            break
        if lc and lc.get("result") == "rejected":
            retries += 1
            time.sleep(0.5)
            continue
        break
    idle = ctx.rec.wait_for(lambda s: s.get("state") == "IDLE" and not s.get("estop"), 3.0)
    substep(rec, "reset", reset_ok and bool(idle), "reset ok=%s after %d 'retry' rejections; state %s" % (reset_ok, retries, (idle or {}).get("state")))
    # 6. resume: explicit approved move back to READY (not an automatic recovery)
    ok, lc, plan, ex, st = ctx.bridge_motion(rec, {"cmd": "move_named", "name": "READY", "speed": ctx.args.speed}, 90.0)
    resumed = ok and ctx.at(ready["joints_deg"])
    substep(rec, "resume_after_reset", resumed, "result %s state %s" % (json.dumps(lc), (st or {}).get("state")))
    snap = ctx.watch.snapshot()
    rec["motion_observed"] = snap
    rec["stages"] = {"plan": "attempted", "execution": "attempted"}
    rec["plan_success"] = True
    rec["execution_success"] = bool(es) and preempted and stopped and resumed
    checks = [bool(es), preempted, stopped, hold, rej, reset_ok and bool(idle), resumed]
    actual = "ESTOP in %.2fs, goal %s, stopped in %.2fs, hold %s, rejected %s, reset %s (retries %d), resumed %s" % (
        t_state, (stx.get("exec") or {}).get("goal_status"), t_stop, hold, rej, reset_ok, retries, resumed)
    if all(checks):
        return finish(rec, t0, "PASS", actual)
    failed = [s["name"] for s in rec["substeps"] if not s["ok"]]
    return finish(rec, t0, "FAIL", actual, "expectations not met: %s" % ", ".join(failed), failed[0] if failed else None)


def test_d6(ctx):
    rec = new_record("D6")
    t0 = time.time()
    if not ctx.preflight(rec, need_scene=True):
        return finish(rec, t0, "SKIP", "preflight failed", rec["substeps"][-1]["detail"])
    req = (ctx.info().get("task_requirements") or {}).get("barista") or {}
    if req.get("missing"):
        return finish(rec, t0, "SKIP", "task 'barista' needs taught poses: %s" % ", ".join(req["missing"]))
    if not ctx.approve(rec, "D6"):
        return finish(rec, t0, "SKIP", "motion test not approved", "motion not approved")
    n = max(1, int(ctx.args.barista_cycles))
    cycles = []
    rec["cycles"] = cycles
    ctx.watch.start()
    for k in range(n):
        if ctx.abort:
            break
        c = {"cycle": k + 1, "start_time": now_iso(), "result": None, "duration_sec": None, "failure_step": None,
             "error_message": None, "phase": None, "plan_success": None, "execution_success": None, "steps": []}
        cid, lc, st, plan, ex = ctx.bridge.call({"cmd": "run_task", "task": "barista", "speed": ctx.args.speed}, 600.0)
        rec["bridge"]["cmd_ids"].append(cid)
        task = (st or {}).get("task") or {}
        c["result"] = task.get("result") or ((lc or {}).get("result") if lc else "timeout")
        c["duration_sec"] = task.get("last_cycle_time_s")
        c["steps"] = task.get("steps") or []
        failed = [(i, s) for i, s in enumerate(c["steps"]) if s.get("state") == "failed"]
        if failed:
            i, s = failed[0]
            c["failure_step"] = "%d:%s" % (i + 1, s.get("name"))
            c["error_message"] = task.get("message") or (st or {}).get("last_error")
            if s.get("type") == "gripper":
                c["phase"] = "gripper"
            elif plan and plan.get("attempted") and plan.get("ok") is False:
                c["phase"] = "planning"
            else:
                c["phase"] = "execution"
        obj = ((st or {}).get("object") or {}).get("cup") or {}
        c["object_at_place"] = obj.get("at_place")          # None when the object is not tracked
        c["object_attached_after"] = obj.get("attached")
        if c["result"] == "ok" and c["object_at_place"] is False:
            c["result"] = "failed"
            c["failure_step"] = c["failure_step"] or "VERIFY_PLACE"
            c["error_message"] = "task reported ok but the cup is not on the place pad"
        c["plan_success"] = c["result"] == "ok" or c["phase"] not in ("planning",)
        c["execution_success"] = c["result"] == "ok"
        cycles.append(c)
        rospy.loginfo("safety_tests: barista cycle %d/%d -> %s (%s s) %s", k + 1, n, c["result"], c["duration_sec"], c["failure_step"] or "")
        if c["result"] != "ok" and not ctx.args.continue_on_failure:
            rec["notes"].append("stopped after the first failed cycle (no automatic retry, no recovery motion); remaining cycles skipped")
            break
    snap = ctx.watch.snapshot()
    rec["motion_observed"] = snap
    good = sum(1 for c in cycles if c["result"] == "ok")
    rec["stages"] = {"plan": "attempted", "execution": "attempted"}
    rec["plan_success"] = all(c["plan_success"] for c in cycles) if cycles else None
    rec["execution_success"] = all(c["execution_success"] for c in cycles) if cycles else None
    durations = [c["duration_sec"] for c in cycles]
    actual = "%d/%d cycles ok, durations %s s" % (good, n, durations)
    rec["barista_summary"] = {"successful_cycles": good, "total_cycles": n, "executed_cycles": len(cycles), "cycle_durations": durations}
    if cycles and good == len(cycles) == n:
        return finish(rec, t0, "PASS", actual)
    first_bad = next((c for c in cycles if c["result"] != "ok"), None)
    return finish(rec, t0, "FAIL", actual, (first_bad or {}).get("error_message") or "not all cycles completed",
                  (first_bad or {}).get("failure_step"))


TEST_FUNCS = {"D1": test_d1, "D2": test_d2, "D3": test_d3, "D4": test_d4, "D5": test_d5, "D6": test_d6}


# ============================================================ runner
def environment(ctx, report):
    info = ctx.info()
    st = ctx.status()
    report.data["environment"].update({
        "ros_distro": os.environ.get("ROS_DISTRO"), "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
        "hostname": socket.gethostname(), "python": platform.python_version(),
        "bridge": {"alive": ctx.rec.alive(), "cmd_id_tagging": "cmd_id" in (st.get("plan") or {}),
                   "state_at_start": st.get("state"), "speed_pct": st.get("speed"),
                   "workcell_loaded": info.get("workcell_loaded"), "taught_poses": sorted((info.get("taught_poses") or {}).keys()),
                   "task_requirements": info.get("task_requirements"), "joints": info.get("joints"),
                   "estop_is_simulation_only": True},
        "bridge_params": ctx.params,
        "moveit": {"planning_frame": ctx.probe.group.get_planning_frame() if ctx.probe else None, "eef_link": EEF_LINK},
        "controller": CONTROLLER})


def run_tests(ctx, report, test_ids, status_cb=None):
    for tid in test_ids:
        if ctx.abort:
            break
        if status_cb:
            status_cb(tid)
        rospy.loginfo("safety_tests: === %s %s ===", tid, TEST_INFO[tid][1])
        try:
            rec = TEST_FUNCS[tid](ctx)
        except KeyboardInterrupt:
            ctx.bridge.send({"cmd": "stop"})
            rec = finish(new_record(tid), time.time(), "FAIL", "interrupted by user (stop sent, no recovery motion)", "interrupted by user")
            report.add(rec)
            raise
        except Exception as exc:
            rospy.logerr("safety_tests: %s crashed: %s\n%s", tid, exc, traceback.format_exc())
            rec = finish(new_record(tid), time.time(), "FAIL", "runner exception: %s" % exc, str(exc), "runner")
        report.add(rec)
        print("  %s %s -> %s: %s" % (tid, rec["name"], rec["result"], (rec.get("actual_behavior") or "")[:120]))
        if status_cb:
            status_cb(None)


class Serve(object):
    """Pendant panel mode: /pendant/test_cmd -> runs, /pendant/test_status <- JSON status."""

    def __init__(self, ctx, make_report):
        self.ctx = ctx
        self.make_report = make_report
        self.pub = rospy.Publisher("/pendant/test_status", String, queue_size=1, latch=True)
        self.report = None
        self.running = False
        self.current = None
        self.thread = None
        rospy.Subscriber("/pendant/test_cmd", String, self.on_cmd, queue_size=5)
        rospy.Timer(rospy.Duration(1.0), lambda _e: self.publish())
        self.publish()

    def publish(self):
        data = {"runner_alive": True, "mode": "serve", "running": self.running, "current_test": self.current,
                "approve_motion": bool(self.ctx.args.approve_motion),
                "available": [{"test_id": t[0], "name": t[1], "motion": t[2], "description": t[3]} for t in TESTS],
                "tests": self.report.data["tests"] if self.report else [], "summary": self.report.data["summary"] if self.report else {},
                "report_path": self.report.json_path if self.report else None, "stamp": time.time()}
        self.pub.publish(String(data=json.dumps(data, default=str)))

    def on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        if cmd.get("cmd") == "abort_tests":
            self.ctx.abort = True
            self.ctx.bridge.send({"cmd": "stop"})
            return
        if cmd.get("cmd") != "run_tests" or self.running:
            return
        st = self.ctx.status()
        if st.get("estop") or st.get("state") in ("PLANNING", "EXECUTING") or (st.get("task") or {}).get("running"):
            rospy.logwarn("safety_tests: run_tests refused — bridge busy or E-STOP active")
            return
        ids = [t for t in (cmd.get("tests") or []) if t in TEST_FUNCS]
        if not ids:
            return
        self.ctx.args._serve_approved = bool(cmd.get("approve_motion")) and bool(self.ctx.args.approve_motion)
        if cmd.get("barista_cycles"):
            self.ctx.args.barista_cycles = int(cmd["barista_cycles"])
        self.thread = threading.Thread(target=self._run, args=(ids,), daemon=True)
        self.thread.start()

    def _run(self, ids):
        self.running = True
        self.ctx.abort = False
        self.report = self.make_report()
        environment(self.ctx, self.report)
        try:
            run_tests(self.ctx, self.report, ids, status_cb=self._status)
        finally:
            self.ctx.bridge.send({"cmd": "set_speed", "speed": self.ctx.initial_speed or 30})
            self.running = False
            self.current = None
            self.report.print_summary()
            self.publish()

    def _status(self, tid):
        self.current = tid
        self.publish()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tests", default="", help="comma-separated ids, e.g. D2,D3,D4")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--approve-motion", action="store_true", help="allow motion tests (each one still asks for 'yes')")
    ap.add_argument("--barista-cycles", type=int, default=5)
    ap.add_argument("--continue-on-failure", action="store_true")
    ap.add_argument("--via-bridge", action="store_true", help="D4: also send the invalid goal through the bridge (needs approval)")
    ap.add_argument("--speed", type=int, default=20, help="bridge speed percent for motion tests")
    ap.add_argument("--no-motion-tol-deg", type=float, default=0.3)
    ap.add_argument("--out", default=os.path.join(rospkg.RosPack().get_path("orion5_interface"), "test_reports"))
    ap.add_argument("--serve", action="store_true", help="pendant panel mode (/pendant/test_cmd, /pendant/test_status)")
    args = ap.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.list:
        for t in TESTS:
            print("%s  %-20s %-8s %s" % (t[0], t[1], "motion" if t[2] else "planning", t[3]))
        return 0
    ids = [t[0] for t in TESTS] if args.all else [s.strip().upper() for s in args.tests.split(",") if s.strip()]
    bad = [i for i in ids if i not in TEST_FUNCS]
    if bad:
        print("unknown test ids: %s" % bad)
        return 2
    if not ids and not args.serve:
        ap.print_help()
        return 2

    moveit_commander.roscpp_initialize([])
    rospy.init_node("safety_tests", anonymous=True)
    ctx = Ctx(args)
    if not ctx.rec.wait_info(15.0) or not ctx.watch.wait_joint_states(5.0):
        print("setup error: /pendant/status, /pendant/robot_info or /joint_states not available (is mery6_simulation.launch running?)")
        return 2
    try:
        cfg = {"ik_attempts": 3, "ik_timeout_s": 0.3, "cartesian_eef_step": 0.005, "cartesian_max_joint_jump_rad": 0.35}
        ready = ctx.taught("READY")
        pos, _ = ctx.watch.current()
        ctx.probe = Probe(cfg, ready["joints_deg"] if ready else rad2deg(pos))
    except Exception as exc:
        print("setup error: MoveIt services not reachable: %s" % exc)
        return 2
    ctx.initial_speed = ctx.status().get("speed")
    print("safety_tests: bridge state %s, taught %s, scene %s, approve_motion=%s" % (
        ctx.status().get("state"), sorted((ctx.info().get("taught_poses") or {}).keys()), ctx.info().get("workcell_loaded"), args.approve_motion))

    def make_report():
        return Report(args.out, rospy.myargv(argv=sys.argv)[1:], args.approve_motion)

    if args.serve:
        Serve(ctx, make_report)
        print("safety_tests: serve mode — waiting for /pendant/test_cmd (motion tests %s)" % ("enabled by --approve-motion" if args.approve_motion else "DISABLED"))
        rospy.spin()
        return 0

    report = make_report()
    environment(ctx, report)
    code = 0
    try:
        run_tests(ctx, report, ids)
    except KeyboardInterrupt:
        print("interrupted")
        code = 1
    finally:
        if ctx.initial_speed is not None:
            ctx.bridge.send({"cmd": "set_speed", "speed": ctx.initial_speed})
        report.write()
        report.print_summary()
    if report.data["summary"]["fail"]:
        code = 1
    moveit_commander.roscpp_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main() or 0)
