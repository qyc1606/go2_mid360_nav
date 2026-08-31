cmake_minimum_required(VERSION 3.0.2)

project(go2_localization_guard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  go2_localization_guard_node
  src/localization_guard.cpp
)

target_link_libraries(
  go2_localization_guard_node
  ${catkin_LIBRARIES}
)