cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(Globals LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} STATIC
        src/Config.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE CONFDIR="${CMAKE_SOURCE_DIR}/conf/"
    PRIVATE DATADIR="/home/arc/.config/${CMAKE_PROJECT_NAME}/conf/"
)
