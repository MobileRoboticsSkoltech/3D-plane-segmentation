cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(App LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(${PROJECT_NAME}
    main.cpp
    app.h
    app.cpp
    app.ui
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Globals
    Utils
    Scenes
    Gui
    Rep
    Control
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE APP_AUTHOR="\\"Arindam\ Roychoudhury\\""
    PRIVATE APP_NAME="\\"App\\""
)

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/styles.css
            ${CMAKE_CURRENT_BINARY_DIR}/styles.css
)
