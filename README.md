
# Mery6 / Orion5 Simulation Validation Platform (ROS Noetic)

Simulation-only platform: **Web pendant → rosbridge → ROS Noetic → MoveIt / ros_control → Gazebo**, with RViz for
planning/validation. No EtherCAT/SOEM layer is used (the `soem_*` packages are untouched and not part of this flow).

```
sudo apt install ros-noetic-rosbridge-server ros-noetic-moveit-commander python3-urdfdom-py
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
| Task definition (14 steps) | `orion5_interface/config/barista_task.yaml` |
| Taught poses (written by the pendant **Teach** button) | `orion5_interface/config/taught_poses.yaml` |
| Verification helper | `rosrun orion5_interface pendant_status.py [keys…] | --watch task | --info` |

Teach flow (no pose is hard-coded): on the pendant Jog page move the TCP with joint jog and **Cartesian jog**
(collision-checked), then type a name and press **Teach**. The Barista task needs `READY`, `PRE_PICK`, `PICK`,
`PRE_PLACE`, `PLACE`; recommended order: `PICK` → Cartesian +Z 50–80 mm → `PRE_PICK` → `PLACE` → +Z → `PRE_PLACE` →
`READY` (a non-singular pose above the table). The Run page shows which poses are still missing and refuses to start
until all exist. Approach/retract steps are straight-line (`cartesian_to`, 100 % path required); transfers are
MoveIt joint-space plans.

Simulation limitations (also published in `/pendant/robot_info.limitations`):
- No grasp/attach plugin: the cup is **not** attached to the gripper; `GRIPPER_CLOSE` only verifies the finger
  angle and `TRANSFER` moves an empty gripper. The cup is not a planning-scene obstacle (it is the target).
- The MC1 links have no collision geometry; an approximate box attached to `eef_link` in the planning scene stands
  in for the gripper (planning only, not Gazebo physics; open fingers along ±x are not covered).
- Joint jog bypasses MoveIt (no collision check); use Cartesian jog for the final approach while teaching.
- `HOME` (all joints 0) is an elbow + wrist singularity: start Cartesian moves from `READY`.
- E-STOP during `PLANNING` takes effect when the planner returns (≤ 5 s); the trajectory is never sent.

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
