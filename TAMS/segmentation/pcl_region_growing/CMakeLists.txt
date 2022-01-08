#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)

include_directories(include)

add_library (region_growing src/region_growing.cpp)
target_link_libraries(region_growing pcl_common pcl_io pcl_features pcl_search pcl_filters pcl_visualization ${PCL_LIBRARIES})

add_executable (region_growing_test src/region_growing.cpp src/main.cpp)
target_link_libraries(region_growing_test pcl_common pcl_io pcl_features pcl_search pcl_filters pcl_visualization ${PCL_LIBRARIES})

