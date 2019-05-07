cmake_minimum_required(VERSION 2.8.3)
project(hopper_laser)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_assembler
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  message_generation
)

add_service_files(
  FILES
  HopperScan.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
