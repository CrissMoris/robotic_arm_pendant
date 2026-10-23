#!/usr/bin/python3

import math
import rospy
from mcfly_grippers.srv import GripperControl, GripperControlResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

"""
 @author: Mehmet Kahraman
 @date: 13.11.2024
 @about: Gripper Controller ROS Class Service
"""

class RosGripperControl():
    
    def __init__(self, gripper_name, gripper_joints, controller_name, angle_limit):
        
        self.gripper_name = gripper_name
        self.gripper_joints = gripper_joints
        self.controller_name = controller_name
        self.angle_limit = angle_limit
    
        self.init_ros()
    
    def init_ros(self):
        rospy.loginfo(self.gripper_name + " gripper service initializing...")
        
        service_topic = "/mcfly_grippers/" + self.gripper_name + "/angle"
        self.service = rospy.Service(service_topic, GripperControl, self.service_callback)
        
        traj_control_topic = "/" + self.controller_name + "/command"
        self.traj_publisher = rospy.Publisher(traj_control_topic, JointTrajectory, queue_size=10)
        
        rospy.sleep(1)
    
    def service_callback(self, req):
        
        if abs(req.angle) > abs(self.angle_limit):
            rospy.loginfo("Gripper angle limit is " + str(self.angle_limit) + " degrees.")
            return GripperControlResponse("Limit exceeded.", False)
        
        if req.time <= 0.0:
            message = "Time must be greater than 0.0 sec."
            rospy.loginfo(message)
            return GripperControlResponse(message, False)

        rospy.loginfo(self.gripper_name + " gripper is moving to " + str(req.angle) + " degrees in " + str(req.time) + " sec.")
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.gripper_joints
        
        point_msg = JointTrajectoryPoint()
        
        point_msg.positions = [0.0 for i in range(6)]
        point_msg.positions[0] = self.deg_to_rad(req.angle)
        point_msg.positions[1] = self.deg_to_rad(req.angle)
        point_msg.positions[2] = self.deg_to_rad(req.angle)
        point_msg.positions[3] = self.deg_to_rad(req.angle) * -1
        point_msg.positions[4] = self.deg_to_rad(req.angle) * -1
        point_msg.positions[5] = self.deg_to_rad(req.angle) * -1
        point_msg.velocities = [0.0 for i in range(6)]

        point_msg.time_from_start = rospy.Duration(req.time)
        traj_msg.points.append(point_msg)
        
        self.traj_publisher.publish(traj_msg)
        return GripperControlResponse("Successfull.", True)
    
    def deg_to_rad(self, deg):
        return math.radians(deg)
    
    def start_service(self):
        rospy.loginfo(self.gripper_name + " gripper service started.")
        rospy.spin()
