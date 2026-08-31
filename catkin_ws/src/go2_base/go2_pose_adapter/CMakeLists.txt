cmake_minimum_required(VERSION 3.0.2)
project(go2_pose_adapter)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  tf2_ros
  xmlrpcpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(go2_pose_adapter_node src/go2_pose_adapter.cpp)

target_link_libraries(go2_pose_adapter_node ${catkin_LIBRARIES})
