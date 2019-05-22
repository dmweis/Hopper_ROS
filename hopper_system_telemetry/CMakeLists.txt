cmake_minimum_required(VERSION 2.8.3)
project(hopper_system_telemetry)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
