cmake_minimum_required(VERSION 3.0.2)

project(go2_ego_cloud_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
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
  go2_ego_cloud_adapter_node
  src/ego_cloud_adapter.cpp
)

target_link_libraries(
  go2_ego_cloud_adapter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)