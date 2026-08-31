cmake_minimum_required(VERSION 3.0.2)
project(go2_map_accumulator)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_srvs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(go2_map_accumulator_node src/map_accumulator.cpp)

target_link_libraries(
  go2_map_accumulator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
