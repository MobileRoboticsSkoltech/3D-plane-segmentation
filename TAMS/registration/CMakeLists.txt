#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)

include_directories(include)						
set(srcs src/registration.cc)
		
add_library (registration ${srcs})
target_link_libraries(registration pcl_features pcl_search pcl_filters pcl_visualization ${PCL_LIBRARIES})
target_link_libraries(registration common)
