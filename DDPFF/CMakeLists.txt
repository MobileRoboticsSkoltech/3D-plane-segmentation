cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(DDPFF LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenGL_GL_PREFERENCE GLVND)

# Add sub directories
add_subdirectory(globals)
add_subdirectory(control)
add_subdirectory(gui)
add_subdirectory(scenes)
add_subdirectory(utils)
add_subdirectory(rep)
add_subdirectory(app)
