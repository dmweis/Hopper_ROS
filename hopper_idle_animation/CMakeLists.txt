cmake_minimum_required(VERSION 2.8.3)
project(hopper_idle_animation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hopper_controller
  hopper_msgs
  rospy
  std_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
