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

include_directories(src ${CMAKE_CURRENT_SOURCE_DIR}/../../../ns-allinone-3.45/ns-3.45/build/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../ns-allinone-3.45/ns-3.45/build/lib)

add_executable(diff_drive_ns3_ros2 src/diff_drive_ns3_ros2.cpp
               src/diff_drive_robot.cpp)

target_link_libraries(diff_drive_ns3_ros2
  ns3.45-core-default
  ns3.45-network-default
  ns3.45-internet-default
  ns3.45-wifi-default
  ns3.45-mobility-default
  ns3.45-tap-bridge-default
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
