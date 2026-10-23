#!/usr/bin/python3

import rospy
from controller_manager_msgs.srv import SwitchControllerRequest
from std_srvs.srv import TriggerRequest
from PyQt5.QtCore import QThread, pyqtSignal

"""
 @author: Mehmet Kahraman
 @date: 04.04.2024
 @about: Ros Threads for orion 5 arm interface
"""

class ConnectionWorkerThread(QThread):
    
    thread_signal = pyqtSignal(str)
    connection_signal = pyqtSignal(bool)
    
    def __init__(self, enable_state_topic):
        
        QThread.__init__(self)
        
        self.update_sec = 1
        self.enable_state_topic = enable_state_topic
    
    def run(self):
        
        prev_state = False
        curr_state = False
        topics = []
        
        while not rospy.is_shutdown():
            
            curr_state = False
            
            try:
                topics = [i[0] for i in rospy.get_published_topics()]
            except:
                topics = []
            
            for topic in topics:
                if topic == self.enable_state_topic:
                    curr_state = True
            
            if curr_state != prev_state:
                self.connection_signal.emit(curr_state)
                prev_state = curr_state

            rospy.sleep(self.update_sec)
    
class ReceiveErrorWorkerThread(QThread):
    
    thread_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    
    def __init__(self, get_err_src_client, error_message_object):
        
        QThread.__init__(self)
        
        self.get_err_src_client = get_err_src_client
        self.get_err_req = TriggerRequest()
        self.error_message_object = error_message_object
    
    def run(self):
        self.thread_signal.emit("Receiving error code...")
        
        try:
            response = self.get_err_src_client(self.get_err_req)
            
            if response.success == True:
                error_msg = response.message
                error_codes = self.parse_error_message(error_msg)
                self.thread_signal.emit("Error codes: " + ", ".join(error_codes) + " received.")
                error_data = self.get_error_str_data(error_codes)
                self.error_signal.emit(error_data)
                
            else:
                self.thread_signal.emit("Error code is not received !!")
                rospy.logwarn("Success: False, Error code is not received !!")
                
        except:
            rospy.logerr("get_error_code service failed !!")
    
    def parse_error_message(self, error_msg):
        words = error_msg.split()
        error_codes = [word for word in words if word.startswith('0x')]
        return error_codes
    
    def get_unique_errors(self, error_codes):
        uniq_errors = []
        
        for err in error_codes:
            if err not in uniq_errors:
                uniq_errors.append(err)
                
        return uniq_errors.copy()
    
    def get_unique_error_indexes(self, error_codes, unique_errors):
        uniq_error_indexes = []
        temp_list = []
        
        for uniq_err in unique_errors:
            temp_list = []
            for j in range(len(error_codes)):
                if uniq_err == error_codes[j]:
                    temp_list.append("J" + str(j+1))
            uniq_error_indexes.append(temp_list.copy())
        
        return uniq_error_indexes.copy()

    def get_error_str_data(self, error_codes):
        unique_errors = self.get_unique_errors(error_codes)
        uniq_error_indexes = self.get_unique_error_indexes(error_codes, unique_errors)

        data = ""
        for i in range(len(unique_errors)):
            if unique_errors[i] == "0x0":
                continue
            data = data + ", ".join(uniq_error_indexes[i]) + " : " + unique_errors[i] + " --> " + self.error_message_object.get_error_message(unique_errors[i]) + "\n"
        
        if data == "":
            data = "0x0 --> No error"
        
        return data

class ClearFaultWorkerThread(QThread):
    
    thread_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    home_state_signal = pyqtSignal(bool)
    
    def __init__(self, fault_clear_srv_client):
        
        QThread.__init__(self)
        
        self.fault_clear_srv_client = fault_clear_srv_client
        self.fault_clear_req = TriggerRequest()
    
    def run(self):
        self.thread_signal.emit("faults are cleaning...")
        
        try:
            response = self.fault_clear_srv_client.call(self.fault_clear_req)
            self.thread_signal.emit("Fault clear client response message: " + response.message)
            
            if response.success == True:
                self.error_signal.emit("No Fault.")
                self.home_state_signal.emit(False)
            else:
                rospy.logwarn("Success: False, Faults are not cleared !!")
        
        except:
            self.thread_signal.emit('Service Error, clear_fault failed !!')
            rospy.logerr("clear_fault failed !!")

class EnableRobotWorkerThread(QThread):
    
    thread_signal = pyqtSignal(str)
    home_state_signal = pyqtSignal(bool)
    
    def __init__(self, enable_robot_srv_client, switch_service, list_ctrls_srv_client):
        
        QThread.__init__(self)
        
        self.enable_robot_srv_client = enable_robot_srv_client
        self.robot_enable_req = TriggerRequest()
        
        self.switch_service = switch_service
        self.switch_req = SwitchControllerRequest()
        self.list_ctrls_srv_client = list_ctrls_srv_client
        
        self.arm_controller_name = ""
        self.joint_traj_control_name = ""
    
    def enable_robot(self):
        return_value = False
        try:
            response = self.enable_robot_srv_client.call(self.robot_enable_req)
            self.thread_signal.emit("Enable robot client response message: " + response.message)
            
            if response.success == True:
                self.thread_signal.emit("Robot successfully enabled.")
                return_value = True
            else:
                rospy.logwarn("Success: False, Robot is not enabled !!")
        
        except:
            self.thread_signal.emit('Service Error, enable_robot failed !!')
            rospy.logerr("enable_robot failed !!")
        
        return return_value
    
    def start_controller(self):
        if self.arm_controller_name != self.get_running_controller():

            self.switch_req.start_controllers = [self.arm_controller_name]
            self.switch_req.stop_controllers = [self.get_running_controller()]
            self.switch_req.strictness = 1
            
            try:
                switch_result = self.switch_service(self.switch_req)
                
                if switch_result.ok == True:
                    self.thread_signal.emit("Controller switch successfull.")
                    if self.arm_controller_name == self.joint_traj_control_name:
                        self.home_state_signal.emit(True)
                    else:
                        self.home_state_signal.emit(False)
                else:
                    self.thread_signal.emit("Switch to " + self.arm_controller_name + " failed !!")
                    rospy.logerr("Switch to " + self.arm_controller_name + " failed !!")
        
            except:
                self.thread_signal.emit('Service Error, start_controller failed !!')
                rospy.logerr("start_controller failed !!")
        
        else:
            self.thread_signal.emit(self.arm_controller_name + " is already started and running !!")
    
    def get_running_controller(self):
        running_controller_name = ""
        try:
            response = self.list_ctrls_srv_client()
            for controller in response.controller:
                if controller.type == "joint_state_controller/JointStateController":
                    continue
                if controller.state == "running":
                    running_controller_name = controller.name
                    break
        except rospy.ServiceException as e:
            rospy.logerr("list_controllers failed: %s", e)
        
        return running_controller_name
    
    def run(self):
        self.thread_signal.emit("Robot enabling...")
        
        return_value = self.enable_robot()
        if return_value == False:
            return
        
        self.start_controller()
        
        if self.arm_controller_name == self.get_running_controller():
            self.thread_signal.emit("Successfully switched to " + self.arm_controller_name)
        else:
            self.thread_signal.emit("Switch check: controller is not switched !!")

class DisableRobotWorkerThread(QThread):
    
    thread_signal = pyqtSignal(str)
    home_state_signal = pyqtSignal(bool)
    
    def __init__(self, disable_robot_srv_client, switch_service, list_ctrls_srv_client):
        
        QThread.__init__(self)
        
        self.disable_robot_srv_client = disable_robot_srv_client
        self.disable_robot_req = TriggerRequest()
        
        self.switch_service = switch_service
        self.switch_req = SwitchControllerRequest()
        
        self.list_ctrls_srv_client = list_ctrls_srv_client
    
    def disable_robot(self):
        try:
            response = self.disable_robot_srv_client.call(self.disable_robot_req)
            self.thread_signal.emit("Disable robot client response message: " + response.message)
            
            if response.success == True:
                self.thread_signal.emit("Robot successfully disabled.")
                self.home_state_signal.emit(False)
            else:
                rospy.logwarn("Success: False, Faults are not cleared !!")
        
        except:
            self.thread_signal.emit('Service Error, Robot is not disabled !!')
            rospy.logerr("disable_robot failed !!")
    
    def stop_controller(self):
        running_controller = self.get_running_controller()

        if running_controller != "":
            self.switch_req.stop_controllers = [running_controller]
            self.switch_req.strictness = 1
            
            try:
                switch_result = self.switch_service(self.switch_req)
                
                if switch_result.ok == True:
                    self.thread_signal.emit(running_controller + " successfully stopped.")
                else:
                    self.thread_signal.emit(running_controller + " stop failed !!")
                    rospy.logerr(running_controller + " stop failed !!")
            
            except:
                self.thread_signal.emit('Service Error, stop_controller failed !!')
                rospy.logerr("stop_controller failed !!")
        
        else:
            self.thread_signal.emit("No controller is running !!")
    
    def get_running_controller(self):
        running_controller_name = ""
        try:
            response = self.list_ctrls_srv_client()
            for controller in response.controller:
                if controller.type == "joint_state_controller/JointStateController":
                    continue
                if controller.state == "running":
                    running_controller_name = controller.name
                    break
        except rospy.ServiceException as e:
            rospy.logerr("list_controllers failed: %s", e)
        
        return running_controller_name

    def run(self):
        self.thread_signal.emit("Robot disabling...")
        
        self.disable_robot()
        self.stop_controller()
