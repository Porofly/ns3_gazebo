cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ns3_wifi_tap_test)

set(CMAKE_BUILD_TYPE Debug)
# Set C++ standard to C++20 for NS-3 3.45 compatibility
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add safe compilation flags to avoid illegal instruction
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -march=x86-64 -mtune=generic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O1 -march=x86-64 -mtune=generic")

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
    ns3.45-csma-${NS3_PROFILE}
)

# Verify all required libraries exist
foreach(lib ${NS3_LIBRARIES})
    if(NOT EXISTS "${NS3_LIBRARY_DIR}/lib${lib}.so")
        message(WARNING "Library lib${lib}.so not found, but continuing...")
    endif()
endforeach()

include_directories(src ${NS3_INCLUDE_DIR})
link_directories(${NS3_LIBRARY_DIR})

add_executable(ns3_wifi_tap_test ns3_wifi_tap_test.cpp)
target_link_libraries(ns3_wifi_tap_test ${NS3_LIBRARIES})

