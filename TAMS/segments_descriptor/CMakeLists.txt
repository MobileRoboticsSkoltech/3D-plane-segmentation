project(segments_descriptor)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(include)
                    
#add_executable(SegmenstDescriptor src/segments_descriptor.cpp)
#target_link_libraries(SegmenstDescriptor ${PCL_LIBRARIES})
