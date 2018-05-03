cmake_minimum_required(VERSION 2.8.3)
project(hopper_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  hopper_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
