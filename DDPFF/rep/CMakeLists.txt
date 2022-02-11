cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(Rep LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#Add a library with the above sources
add_library(${PROJECT_NAME} STATIC
    src/DDPFF.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    Utils
    Globals
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE RESULTDIR="\\"/home/arc/.config/planelab/conf/\\""
)
