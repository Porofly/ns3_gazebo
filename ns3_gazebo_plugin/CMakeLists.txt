cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(src ${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build/include)
link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build/lib)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(diff_drive_ns3_gazebo SHARED ns3_gazebo_world.cpp)
target_link_libraries(diff_drive_ns3_gazebo ${GAZEBO_LIBRARIES}
  ns3.45-core-default
  ns3.45-network-default
  ns3.45-internet-default
  ns3.45-wifi-default
  ns3.45-mobility-default
  ns3.45-tap-bridge-default
)

