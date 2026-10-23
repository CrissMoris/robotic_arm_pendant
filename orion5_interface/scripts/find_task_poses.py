#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
find_task_poses.py — MoveIt-based candidate search for the Barista poses
PICK / PRE_PICK / PLACE / PRE_PLACE (SIMULATION HELPER, planning only).

What it does
  * reads the workcell (cup, place_pad) from barista.world and the taught READY pose from taught_poses.yaml
  * builds tool-down TCP targets at the cup (mid-height grasp) and above the place pad, rotated about base Z
  * for every candidate: MoveIt IK (seeded with READY, collision-aware, inside URDF joint limits),
    /check_state_validity against the current planning scene (workcell objects + gripper envelope),
    the PRE_* pose straight above it, the straight-line PRE -> target path (compute_cartesian_path fraction and
    joint-jump guard, the same criterion the task runner uses), an optional OMPL plan READY -> PRE_*,
    the joint distance from READY and whether the elbow/wrist branch matches READY
  * prints a ranked report and writes config/pose_candidates.yaml (NOT taught_poses.yaml)

What it does NOT do
  * it never moves the robot and never writes taught_poses.yaml. The operator drives the robot to a chosen
    candidate through the pendant bridge (command printed at the end), checks it in Gazebo/RViz and presses
    Teach. The cup stays unattached (no grasp simulation).

Run while mery6_simulation.launch is up (move_group + pendant_bridge, so the planning scene is loaded):
  rosrun orion5_interface find_task_poses.py [--config pose_search.yaml] [--taught taught_poses.yaml]
                                             [--out pose_candidates.yaml] [--no-write] [--top 3]
"""

import argparse
import copy
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import rospy
import rospkg
import yaml
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetStateValidity, GetStateValidityRequest
import tf.transformations as tft

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
PLANNING_GROUP = "orion5_arm"
EEF_LINK = "eef_tcp_link"
REF_FRAME = "base_link"


def pkg_path(pkg):
    return rospkg.RosPack().get_path(pkg)


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def read_workcell(world_file, z_off):
    """cup centre / size and place_pad top from the SDF world (base_link frame)."""
    root = ET.parse(world_file).getroot()
    out = {}
    for model in root.iter("model"):
        name = model.get("name")
        if name not in ("cup", "place_pad"):
            continue
        pose = model.find("pose")
        x, y, z = [float(v) for v in (pose.text.split() if pose is not None else "0 0 0".split())[:3]]
        geom = model.find(".//collision/geometry")
        if name == "cup":
            cyl = geom.find("cylinder")
            out["cup"] = {"xyz": [x, y, z - z_off], "radius": float(cyl.find("radius").text),
                          "height": float(cyl.find("length").text)}
        else:
            box = geom.find("box")
            sz = [float(v) for v in box.find("size").text.split()]
            out["place_pad"] = {"xyz": [x, y, z - z_off], "size": sz, "top_z": z - z_off + sz[2] / 2.0}
    return out


def quat_z_axis(q):
    return tft.quaternion_matrix(q)[:3, 2]


def pose_from(xyz, q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = xyz
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def tcp_dict(xyz, q):
    r, p, y = tft.euler_from_quaternion(q)
    return {"x": round(xyz[0], 4), "y": round(xyz[1], 4), "z": round(xyz[2], 4),
            "roll": round(math.degrees(r), 2), "pitch": round(math.degrees(p), 2), "yaw": round(math.degrees(y), 2),
            "q": [round(float(v), 6) for v in q], "frame": REF_FRAME}


class PoseFinder(object):

    def __init__(self, cfg, ready_deg):
        self.cfg = cfg
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)
        self.group.set_end_effector_link(EEF_LINK)
        self.group.set_pose_reference_frame(REF_FRAME)
        self.group.set_planning_time(3.0)
        self.group.set_num_planning_attempts(2)
        rospy.wait_for_service("/compute_ik", timeout=15.0)
        rospy.wait_for_service("/check_state_validity", timeout=15.0)
        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self.valid_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        self.bounds = {}
        for n in ARM_JOINTS:
            try:
                lo, hi = self.robot.get_joint(n).bounds()
            except Exception:
                lo, hi = -math.pi, math.pi
            self.bounds[n] = (lo, hi)
        self.ready_rad = [math.radians(v) for v in ready_deg]
        self.base_state = self.robot.get_current_state()   # keeps attached objects (gripper envelope)
        scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.5)
        self.scene_objects = sorted(scene.get_known_object_names())
        self.attached = sorted(scene.get_attached_objects().keys())

    # ---- helpers ---------------------------------------------------------
    def state_with(self, joints_rad):
        st = copy.deepcopy(self.base_state)
        names = list(st.joint_state.name)
        pos = list(st.joint_state.position)
        for n, v in zip(ARM_JOINTS, joints_rad):
            if n in names:
                pos[names.index(n)] = v
            else:
                names.append(n)
                pos.append(v)
        st.joint_state.name = names
        st.joint_state.position = pos
        st.joint_state.velocity = []
        st.joint_state.effort = []
        return st

    def within_limits(self, joints_rad):
        for n, v in zip(ARM_JOINTS, joints_rad):
            lo, hi = self.bounds[n]
            if not (lo - 1e-6 <= v <= hi + 1e-6):
                return False
        return True

    def ik(self, pose, seed_rad):
        req = GetPositionIKRequest()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.ik_link_name = EEF_LINK
        req.ik_request.robot_state = self.state_with(seed_rad)
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = REF_FRAME
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = rospy.Duration(float(self.cfg.get("ik_timeout_s", 0.2)))
        best = None
        for _ in range(int(self.cfg.get("ik_attempts", 4))):
            resp = self.ik_srv(req)
            if resp.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            js = resp.solution.joint_state
            sol = [js.position[list(js.name).index(n)] for n in ARM_JOINTS]
            if not self.within_limits(sol):
                continue
            d = max(abs(a - b) for a, b in zip(sol, seed_rad))
            if best is None or d < best[0]:
                best = (d, sol)
        return best[1] if best else None

    def validity(self, joints_rad):
        req = GetStateValidityRequest()
        req.robot_state = self.state_with(joints_rad)
        req.group_name = PLANNING_GROUP
        resp = self.valid_srv(req)
        return bool(resp.valid), ["%s<->%s" % (c.contact_body_1, c.contact_body_2) for c in resp.contacts]

    def cartesian_check(self, start_rad, target_pose):
        """Straight-line feasibility start -> target (fraction, max joint jump). Nothing is executed."""
        self.group.set_start_state(self.state_with(start_rad))
        try:
            plan, fraction = self.group.compute_cartesian_path(
                [target_pose], float(self.cfg.get("cartesian_eef_step", 0.005)), avoid_collisions=True)
        finally:
            self.group.set_start_state_to_current_state()
        pts = plan.joint_trajectory.points
        jump = 0.0
        for a, b in zip(pts, pts[1:]):
            jump = max(jump, max(abs(x - y) for x, y in zip(a.positions, b.positions)))
        return round(fraction, 3), round(jump, 3), len(pts)

    def plan_check(self, start_rad, goal_rad):
        """OMPL plan start -> goal (planning only). Returns (ok, planning_time_s)."""
        self.group.set_start_state(self.state_with(start_rad))
        try:
            self.group.set_joint_value_target(goal_rad)
            t0 = time.time()
            res = self.group.plan()
            dt = time.time() - t0
        finally:
            self.group.clear_pose_targets()
            self.group.set_start_state_to_current_state()
        if isinstance(res, tuple):
            ok = bool(res[0]) and len(res[1].joint_trajectory.points) > 0
        else:
            ok = len(res.joint_trajectory.points) > 0
        return ok, round(dt, 2)

    def distance(self, joints_rad):
        d = [math.degrees(a - b) for a, b in zip(joints_rad, self.ready_rad)]
        return round(max(abs(v) for v in d), 1), round(math.sqrt(sum(v * v for v in d)), 1)

    def same_branch(self, joints_rad):
        """Elbow (joint_3) and wrist (joint_5) on the same side as READY, base (joint_1) within 90 deg."""
        def sgn(v):
            return 0 if abs(v) < 0.05 else (1 if v > 0 else -1)
        j, r = joints_rad, self.ready_rad
        return (sgn(j[2]) == sgn(r[2]) or sgn(r[2]) == 0) and (sgn(j[4]) == sgn(r[4]) or sgn(r[4]) == 0) \
            and abs(j[0] - r[0]) < math.pi / 2

    # ---- candidate search ------------------------------------------------
    def search(self, name, pre_name, xyz, q_ref):
        cfg = self.cfg
        results = []
        yaws = cfg.get("yaw_candidates_deg", [0])
        offsets = cfg.get("xy_offsets_m", [[0.0, 0.0]]) or [[0.0, 0.0]]
        dz = float(cfg.get("approach_dz", 0.08))
        for yaw in yaws:
            q = tft.quaternion_multiply(tft.quaternion_about_axis(math.radians(yaw), (0, 0, 1)), q_ref)
            q = [float(v) for v in q]
            for ox, oy in offsets:
                p_xyz = [xyz[0] + ox, xyz[1] + oy, xyz[2]]
                pre_xyz = [p_xyz[0], p_xyz[1], p_xyz[2] + dz]
                cand = {"name": name, "yaw_deg": yaw, "xy_offset": [ox, oy],
                        "tcp": tcp_dict(p_xyz, q), "pre_tcp": tcp_dict(pre_xyz, q),
                        "reachable": False, "valid": None, "contacts": [],
                        "pre_reachable": False, "pre_valid": None, "pre_contacts": [],
                        "cartesian_fraction": None, "cartesian_joint_jump": None, "plan_ready_to_pre": None,
                        "joints_deg": None, "pre_joints_deg": None, "dist_ready_max_deg": None,
                        "dist_ready_norm_deg": None, "same_branch": None, "score": None}
                sol = self.ik(pose_from(p_xyz, q), self.ready_rad)
                if sol is None:
                    results.append(cand)
                    continue
                cand["reachable"] = True
                cand["joints_deg"] = [round(math.degrees(v), 2) for v in sol]
                cand["valid"], cand["contacts"] = self.validity(sol)
                cand["dist_ready_max_deg"], cand["dist_ready_norm_deg"] = self.distance(sol)
                cand["same_branch"] = self.same_branch(sol)
                pre = self.ik(pose_from(pre_xyz, q), sol)
                if pre is not None:
                    cand["pre_reachable"] = True
                    cand["pre_joints_deg"] = [round(math.degrees(v), 2) for v in pre]
                    cand["pre_valid"], cand["pre_contacts"] = self.validity(pre)
                    fr, jump, npts = self.cartesian_check(pre, pose_from(p_xyz, q))
                    cand["cartesian_fraction"], cand["cartesian_joint_jump"] = fr, jump
                    if cfg.get("plan_check", True) and cand["pre_valid"]:
                        ok, dt = self.plan_check(self.ready_rad, pre)
                        cand["plan_ready_to_pre"] = {"ok": ok, "time_s": dt}
                # score: feasibility gates first, then closeness to READY
                gates = [cand["valid"], cand["pre_reachable"], cand["pre_valid"],
                         (cand["cartesian_fraction"] or 0) >= 0.999,
                         (cand["cartesian_joint_jump"] or 9) <= float(cfg.get("cartesian_max_joint_jump_rad", 0.35)),
                         cand["plan_ready_to_pre"] is None or cand["plan_ready_to_pre"]["ok"],
                         cand["same_branch"]]
                cand["feasible"] = all(bool(g) for g in gates[:-1])
                cand["score"] = round((0 if cand["feasible"] else 1000) + (0 if cand["same_branch"] else 200)
                                      + cand["dist_ready_norm_deg"], 1)
                results.append(cand)
        results.sort(key=lambda c: (c["score"] if c["score"] is not None else 9999))
        for i, c in enumerate(results):
            c["rank"] = i + 1
        return results


def fmt_j(j):
    return "[" + ", ".join("%7.1f" % v for v in j) + "]" if j else "—"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(pkg_path("orion5_interface"), "config", "pose_search.yaml"))
    ap.add_argument("--taught", default=os.path.join(pkg_path("orion5_interface"), "config", "taught_poses.yaml"))
    ap.add_argument("--out", default=os.path.join(pkg_path("orion5_interface"), "config", "pose_candidates.yaml"))
    ap.add_argument("--no-write", action="store_true", help="only print, do not write pose_candidates.yaml")
    ap.add_argument("--top", type=int, default=3, help="candidates to print per pose")
    ap.add_argument("--ready-joints-deg", type=float, nargs=6, metavar="DEG",
                    help="use these READY joints instead of taught_poses.yaml (e.g. a validated candidate)")
    args = ap.parse_args(rospy.myargv(argv=sys.argv)[1:])

    moveit_commander.roscpp_initialize([])
    rospy.init_node("find_task_poses", anonymous=True)

    cfg = load_yaml(args.config)
    # --- READY ------------------------------------------------------------
    ready_tcp_q = None
    if args.ready_joints_deg:
        ready_deg = list(args.ready_joints_deg)
        print("READY from command line: %s" % fmt_j(ready_deg))
    else:
        taught = (load_yaml(args.taught).get("taught_poses") or {}) if os.path.isfile(args.taught) else {}
        if "READY" not in taught:
            print("ERROR: no taught READY pose in %s — teach READY from the pendant first "
                  "(or pass --ready-joints-deg)." % args.taught)
            return 2
        ready_deg = [float(v) for v in taught["READY"]["joints_deg"]]
        ready_tcp_q = taught["READY"].get("tcp", {}).get("q")
        print("READY (taught): %s  tcp=%s" % (fmt_j(ready_deg), taught["READY"].get("tcp")))

    # --- workcell targets ---------------------------------------------------
    world = cfg.get("world_file") or os.path.join(pkg_path("orion5_description"), "world", "barista.world")
    z_off = float(cfg.get("gazebo_z_offset", 0.40))
    wc = read_workcell(world, z_off)
    targets = dict(cfg.get("targets") or {})
    if "PICK" not in targets and "cup" in wc:
        cup = wc["cup"]
        frac = float(cfg.get("grasp_height_fraction", 0.5))
        z = cup["xyz"][2] - cup["height"] / 2.0 + frac * cup["height"]
        targets["PICK"] = {"xyz": [cup["xyz"][0], cup["xyz"][1], round(z, 4)], "pre": "PRE_PICK"}
    if "PLACE" not in targets and "place_pad" in wc:
        pad = wc["place_pad"]
        cup_h = wc["cup"]["height"] if "cup" in wc else 0.10
        frac = float(cfg.get("grasp_height_fraction", 0.5))
        z = pad["top_z"] + float(cfg.get("place_clearance", 0.0)) + frac * cup_h
        targets["PLACE"] = {"xyz": [pad["xyz"][0], pad["xyz"][1], round(z, 4)], "pre": "PRE_PLACE"}
    if not targets:
        print("ERROR: no targets (cup/place_pad not found in %s and none configured)" % world)
        return 2
    print("Workcell (%s, z_base = z_gazebo - %.2f): %s" % (os.path.basename(world), z_off, wc))
    print("Targets (base_link): %s" % targets)

    finder = PoseFinder(cfg, ready_deg)
    print("Planning scene objects: %s  attached: %s" % (finder.scene_objects, finder.attached))
    if not finder.scene_objects:
        print("WARNING: planning scene is empty — start mery6_simulation.launch (pendant_bridge loads the workcell)")

    # --- orientation reference ----------------------------------------------
    if ready_tcp_q and quat_z_axis(ready_tcp_q)[2] < -0.9:
        q_ref = [float(v) for v in ready_tcp_q]
        print("Tool-down reference orientation: taught READY TCP (z-axis %s)" % [round(v, 2) for v in quat_z_axis(q_ref)])
    else:
        q_ref = [float(v) for v in tft.quaternion_from_euler(math.pi, 0.0, 0.0)]
        print("Tool-down reference orientation: analytic (roll 180 deg)%s" %
              (" — READY TCP is not tool-down" if ready_tcp_q else ""))

    # --- search -----------------------------------------------------------
    report = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "ready_joints_deg": ready_deg,
              "targets": targets, "scene_objects": finder.scene_objects, "attached": finder.attached,
              "note": "Candidates only. Verify in Gazebo/RViz, move there via the pendant bridge, then Teach. "
                      "No grasp is simulated; the cup stays on the table.",
              "candidates": {}}
    best_cmds = []
    for name, t in targets.items():
        pre_name = t.get("pre", "PRE_" + name)
        print("\n=== %s at %s (and %s %.2f m above) ===" % (name, t["xyz"], pre_name, float(cfg.get("approach_dz", 0.08))))
        res = finder.search(name, pre_name, [float(v) for v in t["xyz"]], q_ref)
        report["candidates"][name] = res
        feasible = [c for c in res if c.get("feasible")]
        print("candidates: %d  reachable: %d  feasible (valid + straight approach%s): %d" % (
            len(res), sum(1 for c in res if c["reachable"]), " + plan" if cfg.get("plan_check", True) else "",
            len(feasible)))
        for c in res[:args.top]:
            print("#%d yaw %5.0f°  reach %s  valid %s  pre_valid %s  cart %s jump %s  plan %s  branch %s  dist max %s° norm %s°" % (
                c["rank"], c["yaw_deg"], c["reachable"], c["valid"], c["pre_valid"], c["cartesian_fraction"],
                c["cartesian_joint_jump"], (c["plan_ready_to_pre"] or {}).get("ok"), c["same_branch"],
                c["dist_ready_max_deg"], c["dist_ready_norm_deg"]))
            print("     %-9s joints %s  tcp xyz (%.3f, %.3f, %.3f) rpy (%.0f, %.0f, %.0f)" % (
                name, fmt_j(c["joints_deg"]), c["tcp"]["x"], c["tcp"]["y"], c["tcp"]["z"],
                c["tcp"]["roll"], c["tcp"]["pitch"], c["tcp"]["yaw"]))
            print("     %-9s joints %s  tcp z %.3f" % (pre_name, fmt_j(c["pre_joints_deg"]), c["pre_tcp"]["z"]))
            if c["contacts"] or c["pre_contacts"]:
                print("     contacts: %s / %s" % (c["contacts"], c["pre_contacts"]))
        if feasible:
            b = feasible[0]
            best_cmds.append((pre_name, b["pre_joints_deg"]))
            best_cmds.append((name, b["joints_deg"]))
        else:
            print("  -> no feasible candidate: adjust yaw_candidates_deg / xy_offsets_m / approach_dz in %s" % args.config)

    if not args.no_write:
        with open(args.out, "w") as f:
            f.write("# Generated by find_task_poses.py — CANDIDATES ONLY, not used by pendant_bridge.py.\n")
            f.write("# Verify each pose in Gazebo/RViz (move there with the pendant), then save it with Teach.\n")
            yaml.safe_dump(report, f, sort_keys=False, allow_unicode=True)
        print("\nCandidates written to %s (taught_poses.yaml untouched)" % args.out)

    if best_cmds:
        print("\nTo verify a candidate in Gazebo, move there through the pendant bridge (collision-checked MoveIt plan),")
        print("watch the robot/RViz, then press Teach on the pendant with the pose name. Suggested order: "
              "PRE_* first, then the pose itself (straight-line Cartesian jog -Z also works for the last cm):")
        for n, j in best_cmds:
            print('  rostopic pub -1 /pendant/cmd std_msgs/String "data: \'{\\"cmd\\":\\"move_joints\\",\\"deg\\":%s,\\"speed\\":20}\'"   # then Teach as %s'
                  % (json_list(j), n))
    moveit_commander.roscpp_shutdown()
    return 0


def json_list(j):
    return "[" + ",".join("%.2f" % v for v in j) + "]"


if __name__ == "__main__":
    sys.exit(main() or 0)
