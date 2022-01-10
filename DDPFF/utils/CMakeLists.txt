cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(Utils LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${ARMADILLO_INCLUDE_DIRS})

# Add a library with the above sources
add_library(${PROJECT_NAME} STATIC
    src/eigenutils.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    Globals
)
