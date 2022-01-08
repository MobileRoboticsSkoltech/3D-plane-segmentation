set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)

include_directories(include)


add_library (octreeRG src/octree_region_growing_segmentation.cc)
target_link_libraries(octreeRG pcl_features pcl_search pcl_filters pcl_visualization ${PCL_LIBRARIES})
target_link_libraries(octreeRG common)



#add_library (area_benchmark src/area_benchmark.cc)
#target_link_libraries(area_benchmark pcl_features pcl_search pcl_filters pcl_visualization ${PCL_LIBRARIES})
#target_link_libraries(area_benchmark common)
