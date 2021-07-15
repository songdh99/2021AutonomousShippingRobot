cmake_minimum_required(VERSION 3.0.2)
project(sensor)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  geometry_msgs
  rospy
  std_msgs
)

add_message_files(
  FILES
  aruco_msgs.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS common_msgs geometry_msgs rospy std_msgs
)
