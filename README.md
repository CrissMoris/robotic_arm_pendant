
# Mery6 / Orion5 Simulation Validation Platform (ROS Noetic)

Simulation-only platform: **Web pendant → rosbridge → ROS Noetic → MoveIt / ros_control → Gazebo**, with RViz for
planning/validation. No EtherCAT/SOEM layer is used (the `soem_*` packages are untouched and not part of this flow).

```
sudo apt install ros-noetic-rosbridge-server ros-noetic-moveit-commander python3-urdfdom-py
cd ~/catkin_ws/src && git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git   # cup attach/detach (Phase E)
cd ~/catkin_ws && catkin_make && source devel/setup.bash
chmod +x src/orion-5/orion5_interface/scripts/pendant_bridge.py
roslaunch orion5_interface mery6_simulation.launch
```

Then open `src/orion-5/mery6_tablet_ui.html` in a browser (tablet or PC), enter the PC IP and press "Bağlan"
(rosbridge websocket port 9090).

| Component | File |
|---|---|
| Single launch | `orion5_interface/launch/mery6_simulation.launch` |
| Robot model (Gazebo **and** MoveIt) | `orion5_description/urdf/orion5_v166_robot_mc1.xacro` (`eef_tcp_link` = MC1 fingertip TCP) |
| Controllers | `joint_state_controller`, `orion_arm_pos_joint_traj_controller`, `mc1_traj_controller` (position interface, no PID) |
| Pendant bridge | `orion5_interface/scripts/pendant_bridge.py` + `config/pendant_bridge.yaml` — JSON on `/pendant/cmd`, `/pendant/status`, `/pendant/robot_info` |
| Web pendant | `mery6_tablet_ui.html` (live `/joint_states` vs. commanded target, URDF joint limits, safe jog, MoveIt moves, gripper, simulation E-STOP, monitor) |

Notes: the web E-STOP is a **simulation** stop (cancels goals, holds controllers, rejects commands), not a
safety-rated E-STOP. Gripper "CLOSED" means the fingers reached the closed angle; no grasp/attach is simulated.

## Barista workcell and task (Phase C)

| Item | File |
|---|---|
| Gazebo workcell (pedestal, table, cup, coffee machine, place pad) | `orion5_description/world/barista.world` (robot spawned at `z:=0.4`) |
| MoveIt planning-scene objects (floor, pedestal, table, coffee_machine, place_pad) + gripper envelope | `orion5_interface/config/pendant_bridge.yaml` (`workcell:`, `gripper_envelope:`), base_link frame, `z_base = z_gazebo - 0.40` |
| Task definition (18 steps, Phase D + E) | `orion5_interface/config/barista_task.yaml` |
| Cup object settings (start/place pose, attachment) | `orion5_interface/config/pendant_bridge.yaml` (`objects:`) |
| Taught poses (written by the pendant **Teach** button) | `orion5_interface/config/taught_poses.yaml` |
| Verification helper | `rosrun orion5_interface pendant_status.py [keys…] | --watch task | --info` |
| Candidate pose search (planning only, never writes taught poses) | `rosrun orion5_interface find_task_poses.py` → `config/pose_candidates.yaml` (inputs: `config/pose_search.yaml`) |
| Safety / motion test runner (Phase D-P2) | `rosrun orion5_interface safety_tests.py` → `orion5_interface/test_reports/` (JSON + CSV) |

Teach flow (no pose is hard-coded): on the pendant Jog page move the TCP with joint jog and **Cartesian jog**
(collision-checked), then type a name and press **Teach**. The Barista task needs `READY`, `PRE_PICK`, `PICK`,
`PRE_PLACE`, `PLACE`; recommended order: `PICK` → Cartesian +Z 50–80 mm → `PRE_PICK` → `PLACE` → +Z → `PRE_PLACE` →
`READY` (a non-singular pose above the table). The Run page shows which poses are still missing and refuses to start
until all exist. Approach/retract steps are straight-line (`cartesian_to`, 100 % path required); transfers are
MoveIt joint-space plans.

Barista sequence (Phase D, 13 steps): `STATE_CHECK` (no motion: fresh joint states, arm stationary, current state
collision-free) → `READY_IF_NEEDED` (moves to READY only if the arm is not already there and no direct collision-checked
plan to PRE_PICK exists; the feasibility plan is never executed) → `PRE_PICK` → `APPROACH_PICK` → `GRIPPER_OPEN` →
`PICK` → `GRIPPER_CLOSE` → `RETRACT_PICK` (straight lift) → `PRE_PLACE` (planner-chosen, collision-checked transfer) →
`PLACE` → `RELEASE` → `RETRACT_PLACE` → `HOME` (mandatory final step). The former TRANSFER return to READY and the
leading HOME/READY moves are gone; the taught pose values are unchanged. A failed or stopped task performs **no
automatic recovery motion**; the operator decides the next command. Status extras: `task.steps[i].note` /
`state: skipped`, `plan.cmd_id` / `plan.attempted`, `exec.cmd_id` / `exec.attempted`, and the read-only command
`{"cmd":"resolve_pose","name":"HOME"}` returns exactly the joints `move_named` would command.

Phase E (cup handling, 18 steps): `SCENE_RESET` puts the cup back at its start pose at the beginning of every cycle
(`/gazebo/set_model_state`; detaches first if needed), `ATTACH_CUP` after `GRIPPER_CLOSE` creates a kinematic fixed
joint between the gripper link and the cup (`gazebo_ros_link_attacher`, refused if the TCP is not at the cup),
`VERIFY_LIFT` after `RETRACT_PICK` measures that the cup rose with the gripper, `DETACH_CUP` after `RELEASE` removes
the joint, and `VERIFY_PLACE` measures that the cup rests on the place pad (tolerance in `objects:`). The pendant
Run page shows the cup state (`attached`, `at_start`, `at_place`, pose) and offers "Sahneyi sıfırla" (`reset_scene`)
and "Bardağı çöz" (`detach`). The bridge status carries `object.cup`. A failed or stopped task never detaches or
moves the cup on its own.

Simulation limitations (also published in `/pendant/robot_info.limitations`):
- The cup is carried by a **kinematic link attachment**, not by finger contact forces (the MC1 fingers have no
  collision geometry): `GRIPPER_CLOSE` verifies the finger angle, `ATTACH_CUP` creates the joint. The transfer is
  measured with `VERIFY_LIFT` / `VERIFY_PLACE`. The cup is not a planning-scene obstacle (it is the target).
- The MC1 links have no collision geometry; an approximate box attached to `eef_link` in the planning scene stands
  in for the gripper (planning only, not Gazebo physics; open fingers along ±x are not covered).
- Joint jog bypasses MoveIt (no collision check); use Cartesian jog for the final approach while teaching.
- `HOME` (all joints 0) is an elbow + wrist singularity: start Cartesian moves from `READY`.
- E-STOP during `PLANNING` takes effect when the planner returns (≤ 5 s); the trajectory is never sent. While the
  worker is still stopping, an E-STOP reset is rejected ("worker still stopping — retry"); the latched ESTOP state is
  never replaced by FAILED/STOPPED, and a reset never resumes a task. The web E-STOP is a simulation stop only.

---

## Safety and motion tests (Phase D-P2)

`orion5_interface/scripts/safety_tests.py` is an independent test runner (it does not live inside the bridge) that
measures the simulation platform's safety behaviour and writes timestamped, never-overwritten reports to
`orion5_interface/test_reports/safety_report_<time>_<pid>.json` + `.csv` (ignored by git).

| id | type | what is verified |
|---|---|---|
| D1 `normal_motion` | motion | plan + execute a known taught pose; joint states reach it within the bridge tolerance |
| D2 `unreachable_target` | planning only | a pose far outside the workspace is rejected (bridge + `/compute_ik`); no trajectory reaches the controller |
| D3 `joint_limit` | planning only | out-of-limit joint targets are rejected by the bridge **before** planning (`plan.attempted == false`); limits read from the URDF; boundary policy checked through the bridge's own validator |
| D4 `collision_check` | planning only | a goal inside the table is invalid in the planning scene (contact pair names `table`, TCP geometrically inside the slab) and cannot be planned; D4b (informative) checks that the planner routes around the coffee machine |
| D5 `estop_during_motion` | motion | simulation E-STOP during a slow move: goal PREEMPTED, joints stop and hold, new moves rejected, reset only after the worker stopped, explicit resume |
| D6 `barista_regression` | motion | N consecutive Barista cycles with per-cycle result, duration, failure step, planning vs execution outcome; no automatic retry |

```
rosrun orion5_interface safety_tests.py --list
rosrun orion5_interface safety_tests.py --tests D2,D3,D4                 # planning only, no approval needed
rosrun orion5_interface safety_tests.py --tests D1,D5 --approve-motion   # asks "yes" before each motion test
rosrun orion5_interface safety_tests.py --tests D6 --approve-motion --barista-cycles 5
```

Rules: motion tests never start without `--approve-motion` plus an interactive confirmation; after a failure, stop or
E-STOP the runner issues **no** recovery motion; a test that could not be executed is reported as SKIP with its reason,
and stages that were not attempted are `null`, never failures. Exit code 1 if any test FAILs. The E-STOP exercised here is
the bridge's simulation stop, not a safety-rated hardware E-STOP; the cup is carried by a kinematic attachment, not a
physics grasp. D6 counts a cycle as successful only if the task result is `ok` and the cup is measured on the place pad.

---

# Orion5 Cartesian Square Trajectory Script

This project provides a ROS/MoveIt Python script for the Orion5 robotic arm that moves the end-effector safely along a square-shaped Cartesian path starting from a given initial pose (specified via command line). After completing the motion, the robot automatically returns to (all joints zeroed) home position. 

# Features

- Flexible start pose specified via terminal.

- Automatic return to home (upright) pose after the square is completed.

- Running parameters and commands easily configurable.


# Requirements

- Operating System: Ubuntu 20.04 

- ROS Distribution: ROS Noetic 

- Python: Python 3 (>=3.6)


*Python Libraries:*

- rospy

- moveit_commander

- geometry_msgs

- copy (standard)

- sys (standard)

*MoveIt! Package:* Must be installed and properly configured.

*Orion URDF:* Orion5_v166.urdf 

# Installation

*rerequisite:* Ensure that Orion5 hardware and MoveIt are fully installed and configured.

Place the square_trajectory.py script into your ROS package or workspace and make it executable with:

```
bash
chmod +x ~/catkin_ws/src/orion-5/orion5_interface/scripts/square_trajectory.py

```

Run the script with the following command:

```
bash
rosrun orion5_interface square_trajectory.py x y z qx qy qz qw duration

```

# Parameters

x, y, z: Metric coordinates of the start position (e.g.: 0.54 0.2 0.24)

qx, qy, qz, qw: Target orientation in quaternion format (e.g.: 1 0 0 0)

duration: Total time for the square trajectory (in seconds, e.g.: 2)

# Example:

```
bash
rosrun orion5_interface square_trajectory.py 0.54 0.2 0.24 1 0 0 0 2
```

# Parameter Configuration

You can customize the core function by modifying these parameters in the script:

- Square Side Length: square_size = 0.2 (default 20cm)

- Safe Heights: z_lift and home_offset (defaults are 0.04m and 0.06m respectively)

- Base Position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] — Zeroes all joints to set the arm upright


# Tips and Warnings

- Before starting, ensure that the start position you provide fits your robot’s simulated/physical workspace and avoids kinematic singularities or reach limits.

- Monitor the terminal to confirm the drawing completed successfully; if the path to any corner cannot be planned, the script will automatically stop and print a warning.

- When running on real hardware, pay special attention to collision limits and safety.

- Make sure all MoveIt configurations and the planning scene are fully loaded before execution.
