cmake_minimum_required(VERSION 3.8)
project(TAMS)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.12.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

link_libraries(stdc++fs)

set(HEADER_FILES src/headers/common.h src/headers/planar_patch.h src/headers/rgb.h src/headers/subwindow.h src/headers/subwindow_region_growing.h src/headers/subwindow_region_growing_parameters.h)
set(SOURCE_FILES src/subwindow_region_growing.cc src/common.cc)

add_executable(TAMS main.cpp ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(TAMS ${PCL_LIBRARIES})