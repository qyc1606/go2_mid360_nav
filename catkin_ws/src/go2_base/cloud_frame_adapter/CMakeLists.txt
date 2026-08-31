cmake_minimum_required(VERSION 3.0.2)

project(cloud_frame_adapter)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  cloud_frame_adapter_node
  src/cloud_frame_adapter.cpp
)

target_link_libraries(
  cloud_frame_adapter_node
  ${catkin_LIBRARIES}
)
