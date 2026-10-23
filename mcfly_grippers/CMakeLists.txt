cmake_minimum_required(VERSION 3.0.2)
project(mcfly_grippers)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

add_service_files(
  FILES
  GripperControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roslib std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
