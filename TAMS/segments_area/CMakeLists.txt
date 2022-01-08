project(segments_area)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(include)						
set(srcs src/segments_area.cc)
		
add_library (segments_area ${srcs})
target_link_libraries(segments_area ${PCL_LIBRARIES})
target_link_libraries(segments_area CGAL CGAL_Core)
target_link_libraries(segments_area common)
