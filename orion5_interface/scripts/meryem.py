#!/usr/bin/python3

import rospy
from Orion5Robot import Orion5Robot

rospy.init_node("orion5_robot_test")
robot = Orion5Robot(mode="sim", enable_on_start=False, moveit_init=True)

num_points = 5
for i in range(num_points):
    print("Point: ", i, " moving...")
    random_point = robot.robot_group.get_random_joint_values()
    robot.go_joint_target(random_point)
    #rospy.sleep(5)
    print("Point: ", i, "done.")
    
print("OK")