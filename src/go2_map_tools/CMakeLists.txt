cmake_minimum_required(VERSION 3.0.2)

project(go2_map_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(
  pcd_to_pgm_node
  src/pcd_to_pgm.cpp
)

target_link_libraries(
  pcd_to_pgm_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
  TARGETS pcd_to_pgm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
