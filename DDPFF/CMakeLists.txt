cmake_minimum_required(VERSION 3.8)
project(DDPFFAdoptation)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
set(BUILD_SHARED_LIBS TRUE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(rep)
add_subdirectory(utils)
add_subdirectory(globals)

add_executable(DDPFFAdoptation main.cpp)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    Utils
    Globals
    Rep
)