cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(Scenes LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenGL_GL_PREFERENCE GLVND)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GLUT REQUIRED)
find_package(Qt5 COMPONENTS Core Xml OpenGL Gui Widgets REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
    src/primitive.cpp
    src/scene.cpp
    src/scenerenderer.cpp
)

target_include_directories(${PROJECT_NAME} 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Xml
    Qt5::OpenGL
    Eigen3::Eigen
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    ${QGLVIEWER_LIBRARY}
    Globals
    Utils
)
