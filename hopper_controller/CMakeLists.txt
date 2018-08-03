cmake_minimum_required(VERSION 2.8.3)
project(hopper_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  hopper_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  BodyMotorPositions.msg
  MotorCompSpeedCommand.msg
  MotorGoalPosition.msg
  TorqueCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
