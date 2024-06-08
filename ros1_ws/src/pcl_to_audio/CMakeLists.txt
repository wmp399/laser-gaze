cmake_minimum_required(VERSION 3.0.2)
project(pcl_to_audio)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
)

catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(pcl_to_audio_node src/pcl_to_audio_node.cpp)

target_link_libraries(pcl_to_audio_node
${catkin_LIBRARIES}
)
