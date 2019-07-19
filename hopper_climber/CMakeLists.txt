cmake_minimum_required(VERSION 2.8.3)
project(hopper_climber)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hopper_controller
  rospy
  std_msgs
  sensor_msgs
  std_srvs
  visualization_msgs
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
