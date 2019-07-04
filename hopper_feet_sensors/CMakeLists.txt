cmake_minimum_required(VERSION 2.8.3)
project(hopper_feet_sensors)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_message_files(
  FILES
  FeetSensorData.msg
)

generate_messages(DEPENDENCIES)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
