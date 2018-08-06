cmake_minimum_required(VERSION 2.8.3)
project(hopper_keep_alive)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

add_service_files(
  FILES
  Halt.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
