cmake_minimum_required(VERSION 2.8.3)
project(hopper_speech)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  hopper_msgs
  sound_play
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY sounds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sounds
)
