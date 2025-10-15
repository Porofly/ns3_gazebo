# https://cmake.org/examples/
cmake_minimum_required(VERSION 3.5)
project(diff_drive_ns3_ros2)

# Set C++ standard to C++20 for NS-3 3.45 compatibility
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(nav_msgs REQUIRED)

# NS-3 path configuration
set(NS3_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../ns-allinone-3.45/ns-3.45)
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
)

include_directories(src ${NS3_INCLUDE_DIR})
link_directories(${NS3_LIBRARY_DIR})

add_executable(diff_drive_ns3_ros2 src/diff_drive_ns3_ros2.cpp
               src/diff_drive_robot.cpp)

target_link_libraries(diff_drive_ns3_ros2
  ${NS3_LIBRARIES}
)

ament_target_dependencies(diff_drive_ns3_ros2
  rclcpp
  urdf
  geometry_msgs
  nav_msgs
)

install(TARGETS
        diff_drive_ns3_ros2
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
