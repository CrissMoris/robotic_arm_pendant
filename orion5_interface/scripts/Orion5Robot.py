#!/usr/bin/python3

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
import math, sys
import moveit_commander
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

"""
 @author: Mehmet Kahraman
 @date: 06.03.2024
 @about: Orion 5 robot soem client ROS class
"""

class Orion5Robot():
    
    def __init__(self, mode, enable_on_start, moveit_init):
        rospy.loginfo("Orion 5 Robot Initializing...")
        self.move_group_name = "orion5_arm"
        self.controller_name = "orion_arm_pos_joint_traj_controller"
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.robot_enable_state = False
        self.robot_fault_state = False
        
        self.home_degree_angles = [0, 0, 0, 0, 0, 0]
        self.joint_bounds = [ [-180,180], [-90,90], [-150,150], [-180,180], [-180,180], [-180,180] ]

        self.max_velocity_ratio = 0.5
        self.max_acceleration_ratio = 0.5
        self.mode = mode
        self.enable_on_start = enable_on_start
        self.moveit_init = moveit_init
        
        if mode == "real":
            self.init_ros()
            if self.enable_on_start == True:
                self.enable_robot()
                
        if self.moveit_init == True:
            self.init_moveit()
        
        rospy.sleep(1)
        rospy.loginfo("Orion 5 Robot is READY.")
    
    def init_ros(self):
        self.fault_clear_req = TriggerRequest()
        self.fault_srv_topic = "/soem_ros_control/ethercat/clear_fault"
        self.fault_clear_srv_client = rospy.ServiceProxy(self.fault_srv_topic, Trigger)
        self.robot_enable_req = TriggerRequest()
        self.enable_srv_topic = "/soem_ros_control/ethercat/enable_robot"
        self.enable_robot_srv_client = rospy.ServiceProxy(self.enable_srv_topic, Trigger)
        self.disable_robot_req = TriggerRequest()
        self.disable_srv_topic = "/soem_ros_control/ethercat/disable_robot"
        self.disable_robot_srv_client = rospy.ServiceProxy(self.disable_srv_topic, Trigger)
        
        self.get_warn_req = TriggerRequest()
        self.warn_topic = "/soem_ros_control/ethercat/get_warning_code"
        self.get_warn_src_client = rospy.ServiceProxy(self.warn_topic, Trigger)
        self.get_err_req = TriggerRequest()
        self.err_topic = "/soem_ros_control/ethercat/get_error_code"
        self.get_err_src_client = rospy.ServiceProxy(self.err_topic, Trigger)
        
        self.switch_srv_topic = "/controller_manager/switch_controller"
        rospy.wait_for_service(self.switch_srv_topic)
        self.switch_service = rospy.ServiceProxy(self.switch_srv_topic, SwitchController)
        
        self.enable_state_topic = "/soem_ros_control/ethercat/enable_state"
        rospy.Subscriber(self.enable_state_topic, Bool, self.enable_sub_callback)
        
        self.fault_state_topic = "/soem_ros_control/ethercat/fault_state"
        rospy.Subscriber(self.fault_state_topic, Bool, self.fault_sub_callback)
        
        self.traj_topic = "/" + self.controller_name + "/command"
        self.traj_publisher = rospy.Publisher(self.traj_topic, JointTrajectory, queue_size=10)
        
        rospy.on_shutdown(self.disable_robot)
    
    def enable_sub_callback(self, msg):
        self.robot_enable_state = msg.data
        
    def fault_sub_callback(self, msg):
        self.robot_fault_state = msg.data
    
    def get_robot_state(self):
        return self.robot_enable_state
    
    def init_moveit(self):
        rospy.loginfo("Moveit initializing...")
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander(self.move_group_name)
        self.robot_group.set_max_velocity_scaling_factor(self.max_velocity_ratio)
        self.robot_group.set_max_acceleration_scaling_factor(self.max_acceleration_ratio)
    
    def adjust_velocity(self, velocity_scaled, acc_scaled):
        self.max_velocity_ratio = velocity_scaled
        self.max_acceleration_ratio = acc_scaled
        self.robot_group.set_max_velocity_scaling_factor(self.max_velocity_ratio)
        self.robot_group.set_max_acceleration_scaling_factor(self.max_acceleration_ratio)
    
    def check_joint_bounds(self, angles):
        new_angles = angles
        for i in range(len(angles)):
            if angles[i] < self.joint_bounds[i][0]:
                new_angles[i] = self.joint_bounds[i][0] + 1
            if angles[i] == self.joint_bounds[i][0]:
                new_angles[i] = self.joint_bounds[i][0] + 1
            if angles[i] > self.joint_bounds[i][1]:
                new_angles[i] = self.joint_bounds[i][1] - 1
            if angles[i] == self.joint_bounds[i][1]:
                new_angles[i] = self.joint_bounds[i][1] - 1
        return new_angles
    
    def enable_robot(self):
        if self.robot_enable_state == False:
            rospy.loginfo("Robot enabling...")
            self.clear_fault_service()
            self.enable_robot_service()
            self.enable_controller_service()
        else:
            rospy.logwarn("Robot already enabled !!")
    
    def clear_fault_service(self):
        rospy.loginfo("Faults are cleaning...")
        try:
            rospy.wait_for_service(self.fault_srv_topic, timeout=0.2)
            response = self.fault_clear_srv_client.call(self.fault_clear_req)
            rospy.loginfo("Fault clear client response message: %s", response.message)
            if response.success == True:
                rospy.loginfo("Faults are cleared.")
            else:
                rospy.logerr("Faults are not cleared !!")
        except:
            rospy.logerr("clear_fault failed !!")
    
    def enable_robot_service(self):
        rospy.loginfo("robot enable service calling...")
        try:
            rospy.wait_for_service(self.enable_srv_topic, timeout=0.2)
            response = self.enable_robot_srv_client.call(self.robot_enable_req)
            rospy.loginfo("Enable robot response message: %s", response.message)
            if response.success == True:
                rospy.loginfo("Robot successfully enabled.")
            else:
                rospy.logerr("Robot is not enabled !!")
        except:
            rospy.logerr('Service Error, enable_robot failed.')
    
    def enable_controller_service(self):
        rospy.loginfo("Controller enabling...")
        if self.robot_enable_state == False:
            rospy.logerr("Robot brakes are not enabled!! Robot is not enabled state!!")
        else:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = [self.controller_name]
            switch_request_object.strictness = 1
            try:
                rospy.wait_for_service(self.switch_srv_topic, timeout=0.2)
                switch_result = self.switch_service(switch_request_object)
                rospy.loginfo("Enable controller response message: %s", switch_result)
                if switch_result.ok == True:
                    rospy.loginfo("Controller successfully started.")
                else:
                    rospy.logerr("Controller is not started !!")
            except:
                rospy.logerr('Service Error, enable_controller failed.')
    
    def disable_robot(self):
        if self.robot_enable_state == True:
            rospy.loginfo("Robot disabling...")
            self.disable_robot_service()
            self.disable_controller_service()
        else:
            rospy.logwarn("Robot already disabled !!")
    
    def disable_robot_service(self):
        rospy.loginfo("robot disable service calling...")
        try:
            rospy.wait_for_service(self.disable_srv_topic, timeout=0.2)
            response = self.disable_robot_srv_client.call(self.disable_robot_req)
            rospy.loginfo("Disable robot response message: %s", response.message)
            if response.success == True:
                rospy.loginfo("Robot successfully disabled.")
            else:
                rospy.logerr("Robot is not disabled !!")
        except:
            rospy.logerr('Service Error, disable_robot failed.')
    
    def disable_controller_service(self):
        rospy.loginfo("Controller disabling...")
        switch_request_object = SwitchControllerRequest()
        switch_request_object.stop_controllers = [self.controller_name]
        switch_request_object.strictness = 1
        try:
            rospy.wait_for_service(self.switch_srv_topic, timeout=0.2)
            switch_result = self.switch_service(switch_request_object)
            rospy.loginfo("Disable controller response message: %s", switch_result)
            if switch_result.ok == True:
                rospy.loginfo("Controller successfully stopped.")
            else:
                rospy.logerr("Controller is not stopped !!")
        except:
            rospy.logerr('Service Error, disable_controller failed.')
    
    def stop_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        self.traj_publisher.publish(traj_msg)
        rospy.loginfo("Trajectory stopped.")
    
    def go_joint_target(self, joint_angles):
        success = False
        string = [str(i) for i in joint_angles]
        rospy.loginfo("Going joint target --> [%s]", " ".join(string))
        joint_angles = self.get_radians_to_execute(joint_angles)
        plan_exec = self.robot_group.go(joint_angles, wait=True)
        if plan_exec != True:
            rospy.logwarn("Plan couldn't be executed")
        else:
            rospy.loginfo("Target positions executed.")
            success = True
        self.robot_group.stop()
        return success
    
    def go_home(self):
        rospy.loginfo("Going home ...")
        home_angles = self.home_degree_angles
        joint_angles = self.get_radians_to_execute(home_angles)
        success = self.go_joint_target(joint_angles)
        return success
    
    def get_radians_to_execute(self, joint_angles):
        angles = self.check_joint_bounds(joint_angles)
        radian_angles = [math.radians(i) for i in angles]
        return radian_angles
    
    def hold_robot_prepared(self):
        rospy.spin()
