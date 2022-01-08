project(loop_detector)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(include)
              
add_library(loop_detector src/loop_detector.cc)
target_link_libraries(loop_detector ${PCL_LIBRARIES})
