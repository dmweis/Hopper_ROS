cmake_minimum_required(VERSION 2.8.3)
project(hopper_emotion_core)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DetectedFace.msg
  Emotions.msg
  FaceAttributes.msg
  FaceRectangle.msg
  FaceDetectionImage.msg
  EmotionChange.msg
  NameAndPersonId.msg
)

add_service_files(
  FILES
  GetPersonIdByName.srv
  GetNameByPersonId.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hopper_controller
   CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
