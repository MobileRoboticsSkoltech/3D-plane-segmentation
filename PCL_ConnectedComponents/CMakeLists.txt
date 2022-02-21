cmake_minimum_required(VERSION 3.8)
project(ConnectedComponents)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.12.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_libraries(stdc++fs)

add_executable(ConnectedComponents main.cpp src/ConnectedComponents.hpp)

target_link_libraries(ConnectedComponents ${PCL_LIBRARIES})