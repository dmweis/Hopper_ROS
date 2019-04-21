cmake_minimum_required(VERSION 2.8.3)
project(hopper_face)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
