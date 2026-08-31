cmake_minimum_required(VERSION 3.0.2)

project(go2_global_planner)


# ============================================================
# Compiler
# ============================================================

add_compile_options(-std=c++14)


# ============================================================
# Catkin dependencies
# ============================================================

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


# ============================================================
# Catkin package
# ============================================================

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)


# ============================================================
# Include directories
# ============================================================

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# ============================================================
# Executable
# ============================================================

add_executable(
  go2_global_planner_node
  src/global_planner.cpp
)


# ============================================================
# Dependencies
# ============================================================

add_dependencies(
  go2_global_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


# ============================================================
# Libraries
# ============================================================

target_link_libraries(
  go2_global_planner_node
  ${catkin_LIBRARIES}
)


# ============================================================
# Install
# ============================================================

install(
  TARGETS go2_global_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)