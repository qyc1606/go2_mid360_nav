
cmake_minimum_required(VERSION 3.0.2)

project(go2_waypoint_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  go2_waypoint_manager_node
  src/waypoint_manager.cpp
)

target_link_libraries(
  go2_waypoint_manager_node
  ${catkin_LIBRARIES}
)