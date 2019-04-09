cmake_minimum_required(VERSION 2.8.3)
project(hopper_open_mct)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hopper_msgs
)


catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
   src/app.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)