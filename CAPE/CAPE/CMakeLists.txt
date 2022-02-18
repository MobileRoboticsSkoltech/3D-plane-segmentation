cmake_minimum_required(VERSION 2.8)

project(CAPE)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PCL COMPONENTS io REQUIRED)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES ${VTK_LIBRARIES} ${QHULL_LIBRARIES} ${FLANN_LIBRARIES}) #we don't need vtk, qhull, flann
message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
message(STATUS "PCL_LIBRARIES=${PCL_LIBRARIES}")


set (CMAKE_CXX_FLAGS "-O2 -g")

set(CMAKE_BUILD_TYPE Release)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(cape_offline
	run_cape_offline.cpp
	CAPE.cpp
	PlaneSeg.cpp
	CylinderSeg.cpp
	Histogram.cpp)

target_link_libraries(cape_offline ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBRARIES})


