
set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)


include_directories(include)
add_executable(subwindow_region_growing src/subwindow_region_growing.cc src/main.cpp)
target_link_libraries(subwindow_region_growing ${PCL_LIBRARIES})
