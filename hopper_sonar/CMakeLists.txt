cmake_minimum_required(VERSION 2.8.3)
project(hopper_sonar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  hopper_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
   src/app.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
