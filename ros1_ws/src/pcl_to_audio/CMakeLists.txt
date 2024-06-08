cmake_minimum_required(VERSION 3.0.2)
project(pcl_to_audio)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_to_audio_node src/pcl_to_audio_node.cpp)

target_link_libraries(pcl_to_audio_node
  ${catkin_LIBRARIES}
)

add_dependencies(pcl_to_audio_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})