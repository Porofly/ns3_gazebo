cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)
# Set C++ standard to C++20 for NS-3 3.45 compatibility
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add safe compilation flags to avoid illegal instruction
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -march=x86-64 -mtune=generic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O1 -march=x86-64 -mtune=generic")

include_directories(src ${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build/lib)

add_executable(ns3_wifi_tap_test ns3_wifi_tap_test.cpp)
target_link_libraries(ns3_wifi_tap_test 
  ns3.45-core-default
  ns3.45-network-default
  ns3.45-internet-default
  ns3.45-wifi-default
  ns3.45-mobility-default
  ns3.45-tap-bridge-default
  ns3.45-csma-default # if you want to see csma work
)

