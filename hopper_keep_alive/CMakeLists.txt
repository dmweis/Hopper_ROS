cmake_minimum_required(VERSION 2.8.3)
project(hopper_keep_alive)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
