cmake_minimum_required(VERSION 3.0.2)

project(go2_cloud_adapter)

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
  go2_cloud_adapter_node
  src/go2_cloud_adapter.cpp
)

target_link_libraries(
  go2_cloud_adapter_node
  ${catkin_LIBRARIES}
)
