cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ns3_mobility)

# Set C++ standard to C++20 for NS-3 3.45 compatibility
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set NS-3 paths
set(NS3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../ns-allinone-3.45/ns-3.45/build/include)
set(NS3_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../ns-allinone-3.45/ns-3.45/build/lib)

include_directories(src ${NS3_INCLUDE_DIR})
link_directories(${NS3_LIBRARY_DIR})

# Find NS-3 library profile (default or optimized)
file(GLOB NS3_CORE_LIB "${NS3_LIBRARY_DIR}/libns3.45-core-*.so")
if(NS3_CORE_LIB)
    string(REGEX MATCH "libns3\\.45-core-([^.]+)\\.so" _ ${NS3_CORE_LIB})
    set(NS3_PROFILE ${CMAKE_MATCH_1})
    message(STATUS "Found NS-3 profile: ${NS3_PROFILE}")
else()
    set(NS3_PROFILE "default")
    message(WARNING "Could not detect NS-3 profile, using default")
endif()

add_executable(ns3_mobility ns3_mobility.cpp)
target_link_libraries(ns3_mobility
  ns3.45-core-${NS3_PROFILE}
  ns3.45-network-${NS3_PROFILE}
  ns3.45-internet-${NS3_PROFILE}
  ns3.45-wifi-${NS3_PROFILE}
  ns3.45-mobility-${NS3_PROFILE}
  ns3.45-tap-bridge-${NS3_PROFILE}
  ns3.45-csma-${NS3_PROFILE}
  ns3.45-bridge-${NS3_PROFILE}
)

