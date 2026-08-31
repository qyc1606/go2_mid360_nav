cmake_minimum_required(VERSION 3.0.2)

project(go2_relocalization_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  go2_relocalization_bridge_node
  src/relocalization_bridge.cpp
)

target_link_libraries(
  go2_relocalization_bridge_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    go2_relocalization_bridge_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
