

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)

include_directories(include)

#set(srcs src/one_pair_tests.cc)
#add_library (planar_segments_matcher ${srcs})
#target_link_libraries(planar_segments_matcher ${PCL_LIBRARIES} pcl_visualization corg)
