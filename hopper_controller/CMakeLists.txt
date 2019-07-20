cmake_minimum_required(VERSION 2.8.3)
project(hopper_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  hopper_msgs
  tf
  tf2
  tf2_ros
  message_generation
  hopper_feet_sensors
)

catkin_python_setup()

add_message_files(
  FILES
  HexapodMotorPositions.msg
  HexapodLegPositions.msg
  LegMotorPositions.msg
  MotorCompliance.msg
  MotorSpeed.msg
  MotorTorque.msg
  SingleLegCommand.msg
  StandCommand.msg
  FoldCommand.msg
)

add_service_files(
  FILES
  ReadHexapodMotorPositions.srv
  ReadHexapodLegPositions.srv
  MoveLegsToPosition.srv
  MoveCoreToPosition.srv
  MoveLegsUntilCollision.srv
  MoveLegsToRelativePosition.srv
  MoveBodyRelative.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
