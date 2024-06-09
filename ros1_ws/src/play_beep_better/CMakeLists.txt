cmake_minimum_required(VERSION 3.0.2)
project(play_beep_better)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
)

add_executable(play_beep_better_node src/play_beep_better_node.cpp)

target_link_libraries(play_beep_better_node
  ${catkin_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
)
