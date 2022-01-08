project(ptulaser)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#set(EXECUTABLE_OUTPUT_PATH bin)
#set(LIBRARY_OUTPUT_PATH lib)
include_directories(include)
		    
add_library(ptulaser src/ptulaser.cpp)
target_link_libraries(ptulaser ptu hokuyo)
add_executable(main src/ptulaser.cpp src/main.cpp)
target_link_libraries(main ptu hokuyo pcl_io pcl_common ${OpenCV_LIBS})

