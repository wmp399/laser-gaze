cmake_minimum_required(VERSION 3.0.2)
project(play_beep)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(SOX REQUIRED sox)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SOX_INCLUDE_DIRS}
)

add_executable(play_beep_node src/play_beep_node.cpp)

target_link_libraries(play_beep_node
  ${catkin_LIBRARIES}
  ${SOX_LIBRARIES}
)
