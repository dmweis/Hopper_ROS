cmake_minimum_required(VERSION 2.8.3)
project(hopper_steamcontroller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  hopper_msgs
  hopper_controller
  hopper_laser
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
