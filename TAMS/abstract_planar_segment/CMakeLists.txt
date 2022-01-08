project(abstract_planar_segment)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(include)
              
add_library(abstract_planar_segment src/abstract_planar_segment.cc)
target_link_libraries(abstract_planar_segment ${PCL_LIBRARIES})
