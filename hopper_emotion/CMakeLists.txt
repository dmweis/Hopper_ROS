cmake_minimum_required(VERSION 2.8.3)
project(hopper_emotion_core)

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
   CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   src/app.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
