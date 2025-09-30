cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(ns3_gazebo_plugin)

# Set C++ standard to C++20 for NS-3 3.45 and Gazebo Harmonic compatibility
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Gazebo Harmonic packages - use find_package without version first
find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_SIM REQUIRED gz-sim8)
pkg_check_modules(GZ_MATH REQUIRED gz-math7)
pkg_check_modules(GZ_PLUGIN REQUIRED gz-plugin2)
pkg_check_modules(SDFORMAT REQUIRED sdformat14)

# NS-3 path configuration
set(NS3_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45)
set(NS3_INCLUDE_DIR ${NS3_BASE_DIR}/build/include)
set(NS3_LIBRARY_DIR ${NS3_BASE_DIR}/build/lib)

# Check if NS-3 directories exist
if(NOT EXISTS ${NS3_INCLUDE_DIR})
    message(FATAL_ERROR "NS-3 include directory not found: ${NS3_INCLUDE_DIR}")
endif()

if(NOT EXISTS ${NS3_LIBRARY_DIR})
    message(FATAL_ERROR "NS-3 library directory not found: ${NS3_LIBRARY_DIR}")
endif()

# Find NS-3 library profile (default or optimized)
file(GLOB NS3_CORE_LIB "${NS3_LIBRARY_DIR}/libns3.45-core-*.so")
if(NS3_CORE_LIB)
    string(REGEX MATCH "libns3\\.45-core-([^.]+)\\.so" _ ${NS3_CORE_LIB})
    set(NS3_PROFILE ${CMAKE_MATCH_1})
    message(STATUS "Found NS-3 profile: ${NS3_PROFILE}")
else()
    message(FATAL_ERROR "NS-3 core library not found in ${NS3_LIBRARY_DIR}")
endif()

# Set required NS-3 libraries with detected profile
set(NS3_LIBRARIES
    ns3.45-core-${NS3_PROFILE}
    ns3.45-network-${NS3_PROFILE}
    ns3.45-internet-${NS3_PROFILE}
    ns3.45-wifi-${NS3_PROFILE}
    ns3.45-mobility-${NS3_PROFILE}
    ns3.45-tap-bridge-${NS3_PROFILE}
    ns3.45-applications-${NS3_PROFILE}
)

include_directories(src ${NS3_INCLUDE_DIR})
link_directories(${NS3_LIBRARY_DIR})

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
  ${NS3_LIBRARIES}
)

# Build the hello world plugin
add_library(hello_world SHARED hello_world.cpp)
target_link_libraries(hello_world
  ${GZ_SIM_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
  ${GZ_PLUGIN_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
)

