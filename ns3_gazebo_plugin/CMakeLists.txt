cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(ns3_gazebo_plugin)

# Set C++ standard to C++20 for NS-3 3.45 and Gazebo Harmonic compatibility
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Gazebo Harmonic packages - use find_package without version first
find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_SIM REQUIRED gz-sim)
pkg_check_modules(GZ_MATH REQUIRED gz-math)
pkg_check_modules(GZ_PLUGIN REQUIRED gz-plugin)
pkg_check_modules(SDFORMAT REQUIRED sdformat)

# NS-3 paths
include_directories(src ${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build/lib)

# Add Gazebo include directories and libraries
include_directories(${GZ_SIM_INCLUDE_DIRS} ${GZ_MATH_INCLUDE_DIRS} ${GZ_PLUGIN_INCLUDE_DIRS} ${SDFORMAT_INCLUDE_DIRS})
link_directories(${GZ_SIM_LIBRARY_DIRS} ${GZ_MATH_LIBRARY_DIRS} ${GZ_PLUGIN_LIBRARY_DIRS} ${SDFORMAT_LIBRARY_DIRS})

# Build the NS3-Gazebo world plugin
add_library(ns3_gazebo_world SHARED ns3_gazebo_world.cpp)
target_link_libraries(ns3_gazebo_world
  ${GZ_SIM_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
  ${GZ_PLUGIN_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
  ns3.45-core-default
  ns3.45-network-default
  ns3.45-internet-default
  ns3.45-wifi-default
  ns3.45-mobility-default
  ns3.45-tap-bridge-default
)

# Build the hello world plugin
add_library(hello_world SHARED hello_world.cpp)
target_link_libraries(hello_world
  ${GZ_SIM_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
  ${GZ_PLUGIN_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
)

