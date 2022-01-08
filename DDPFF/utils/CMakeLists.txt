cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(Utils LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL QUIET COMPONENTS common io)

include_directories(${ARMADILLO_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)

# Add a library with the above sources
add_library(${PROJECT_NAME} SHARED
    src/ColorUtil.cpp
    src/GLlib.cpp
    src/Statistics.cpp
    src/StopWatch.cpp
    src/StopWatchLinux.cpp
    src/Timer.cpp
    src/TimerLinux.cpp
    src/eigenutils.cpp
    src/tum.cpp
    src/bonn.cpp
    src/segcomp.cpp
    src/camerautils.cpp
)

target_sources(${PROJECT_NAME} PRIVATE
    include/utils/Timer.h
    include/utils/TimerLinux.h
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt5::Core
    Qt5::Gui
    Eigen3::Eigen
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    ${ARMADILLO_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    Globals
)
