project(RGSegmentation)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
include_directories(include)

add_executable(RGSegmentation src/region_growing_segmentation.cpp)
target_link_libraries(RGSegmentation ${PCL_LIBRARIES})
