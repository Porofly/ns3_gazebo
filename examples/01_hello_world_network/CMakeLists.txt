# CMakeLists.txt for Hello World Network Example
cmake_minimum_required(VERSION 3.16)

project(hello_world_network)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)

# Find NS-3
pkg_check_modules(NS3 REQUIRED ns3.45-core ns3.45-network ns3.45-wifi ns3.45-internet ns3.45-applications ns3.45-mobility)

# Include directories
include_directories(${NS3_INCLUDE_DIRS})

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NS3_CFLAGS}")

# Create executable
add_executable(hello_world_network hello_world.cc)

# Link libraries
target_link_libraries(hello_world_network ${NS3_LIBRARIES})

# Add library directories
link_directories(${NS3_LIBRARY_DIRS})

# Installation
install(TARGETS hello_world_network
        RUNTIME DESTINATION bin)

install(FILES hello_world.world
        DESTINATION share/hello_world_network)