cmake_minimum_required(VERSION 3.0.2)

project(go2_sdk_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  std_srvs
)

catkin_package()

set(
  UNITREE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../Unitree_SDK2"
  CACHE PATH
  "Path to the Unitree SDK2 source tree"
)

if(NOT EXISTS "${UNITREE_SDK_ROOT}/include/unitree/robot/go2/sport/sport_client.hpp")
  message(FATAL_ERROR
    "Unitree SDK2 was not found at ${UNITREE_SDK_ROOT}. "
    "Set UNITREE_SDK_ROOT to a valid Unitree SDK2 source tree."
  )
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}

  ${UNITREE_SDK_ROOT}/include

  ${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx
  ${UNITREE_SDK_ROOT}/thirdparty/include
)

set(UNITREE_SDK2_LIB
  ${UNITREE_SDK_ROOT}/lib/aarch64/libunitree_sdk2.a
)

set(UNITREE_DDS_LIB
  ${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64/libddsc.so
)

set(UNITREE_DDSXX_LIB
  ${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64/libddscxx.so
)


add_executable(
  go2_sdk_bridge_mock_node
  src/sdk_bridge_mock.cpp
)

target_link_libraries(
  go2_sdk_bridge_mock_node
  ${catkin_LIBRARIES}
)


add_executable(
  go2_sdk_bridge_real_node
  src/sdk_bridge_real.cpp
)

target_link_libraries(
  go2_sdk_bridge_real_node

  ${catkin_LIBRARIES}

  ${UNITREE_SDK2_LIB}
  ${UNITREE_DDSXX_LIB}
  ${UNITREE_DDS_LIB}

  pthread
  dl
  rt
)
