cmake_minimum_required(VERSION 2.8.3)
project(hopper_obstacle_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  octomap_ros
  octomap_msgs
  geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_obstacle_detector src/obstacle_detector.cpp)
target_link_libraries(${PROJECT_NAME}_obstacle_detector ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
