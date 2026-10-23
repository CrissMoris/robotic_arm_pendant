#!/usr/bin/python3

import rospy
from RosGripperControl import RosGripperControl

"""
 @author: Mehmet Kahraman
 @date: 13.11.2024
 @about: Gripper Controller ROS Class Service
"""

if __name__ == '__main__':
    
    rospy.init_node('mc1_gripper_service')
    
    gripper_name = "mc1_gripper"
    gripper_joints = [
        "grip_joint_1_left",
        "grip_joint_2_left",
        "grip_joint_3_left",
        "grip_joint_1_right",
        "grip_joint_2_right",
        "grip_joint_3_right"
    ]
    controller_name = "mc1_traj_controller"
    angle_limit = 45
    
    gripper_control = RosGripperControl(gripper_name, gripper_joints, controller_name, angle_limit)
    
    try:
        gripper_control.start_service()
        
    except rospy.ROSInterruptException:
        print("ROS gripper service stopped.")
