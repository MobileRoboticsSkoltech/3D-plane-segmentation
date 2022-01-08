cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(Control LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
find_package(OpenNI2 REQUIRED)

set(CMAKE_AUTOMOC ON)

# Add a library with the above sources
add_library(${PROJECT_NAME} SHARED
    src/CameraControl.cpp
    src/CameraControlLoop.cpp
    src/CameraLoop.cpp
    src/State.cpp
    src/StateUtil.cpp
)

target_sources(${PROJECT_NAME} PRIVATE
    include/control/CameraControl.h
    include/control/CameraControlLoop.h
    include/control/CameraLoop.h
)


target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt5::Core
    Qt5::Gui
    Eigen3::Eigen
    ${OPENNI2_LIBRARIES}
    Rep
    Scenes
    Utils
    Globals
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE CONFDIR="${CMAKE_SOURCE_DIR}/conf/"
    PRIVATE DATADIR="/home/arc/.config/${CMAKE_PROJECT_NAME}/conf/"
)

