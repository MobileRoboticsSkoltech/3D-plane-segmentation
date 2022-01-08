#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)

include_directories(include)
add_executable(leap_detection src/leap_detection.cc src/leap_detection.cpp)
target_link_libraries(leap_detection ${PCL_LIBRARIES} pcl_visualization)

