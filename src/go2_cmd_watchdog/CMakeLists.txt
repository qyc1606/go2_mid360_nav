cmake_minimum_required(VERSION 3.0.2)

project(go2_cmd_watchdog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  go2_cmd_watchdog_node
  src/cmd_watchdog.cpp
)

target_link_libraries(
  go2_cmd_watchdog_node
  ${catkin_LIBRARIES}
)