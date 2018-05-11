cmake_minimum_required(VERSION 2.8.3)
project(hopper_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
