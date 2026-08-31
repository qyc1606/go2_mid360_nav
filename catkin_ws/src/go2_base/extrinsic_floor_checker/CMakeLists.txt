
cmake_minimum_required(VERSION 3.0.2)

project(extrinsic_floor_checker)



add_compile_options(-std=c++14 -O3)



find_package(catkin REQUIRED COMPONENTS

  roscpp

  sensor_msgs

  pcl_ros

  pcl_conversions

)



find_package(PCL REQUIRED)



catkin_package()



include_directories(

  ${catkin_INCLUDE_DIRS}

  ${PCL_INCLUDE_DIRS}

)



add_executable(

  extrinsic_floor_checker_node

  src/floor_checker.cpp

)



target_link_libraries(

  extrinsic_floor_checker_node

  ${catkin_LIBRARIES}

  ${PCL_LIBRARIES}

)

