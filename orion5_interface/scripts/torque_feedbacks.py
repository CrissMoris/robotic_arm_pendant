#!/usr/bin/python3

"""
 @author: Mehmet Kahraman
 @date: 27.03.2024
 @about: Torque feedback comparison
"""

import rospy, rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pandas as pd

class Listener:
    def __init__(self):
        rospy.init_node('torque_feedbacks')
        
        self.start_time = rospy.Time.now()
        rospy.Subscriber("/joint_states", JointState, self.js_callback)
        rospy.Subscriber("/soem_ros_control/ethercat/torque_sensor", Float64MultiArray, self.torque_sensor_callback)
        
        self.data = []
        self.torque_sensor_data = [0, 0, 0, 0, 0, 0]
        self.columns = ["time", "eff_1", "eff_2", "eff_3", "eff_4", "eff_5", "eff_6", 
                                "sensor_1", "sensor_2", "sensor_3", "sensor_4", "sensor_5", "sensor_6"]
        self.df = pd.DataFrame(columns=self.columns)
        
        pkg_path = rospkg.RosPack().get_path('orion5_interface')
        self.output_csv_path = pkg_path + "/torque_feedbacks.csv"
        
    def js_callback(self, msg):
        time = (rospy.Time.now() - self.start_time).to_sec()
        data = [time]
        for i in range(6):
            data.append(msg.effort[i])
        for i in range(6):
            data.append(self.torque_sensor_data[i])
        self.data.append(data.copy())
    
    def torque_sensor_callback(self, msg):
        self.torque_sensor_data = msg.data

    def create_csv(self):
        for i in range(len(self.data)):
            self.df.loc[len(self.df)] = self.data[i]
        self.df.to_csv(self.output_csv_path, index=False)



if __name__ == '__main__':
    
    print("started.")
    
    try:
        listener = Listener()
        rospy.spin()
        listener.create_csv()
        print("\n ok.")
    
    except:
        print("exited.")
        