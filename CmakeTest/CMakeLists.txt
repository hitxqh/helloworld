cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

project(mainwindow)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Widgets REQUIRED
Qt5Gui REQUIRED
Qt5Core REQUIRED
)
add_executable(mainwindow mainwindow.cpp main.cpp)
target_link_libraries(mainwindow Qt5::Gui Qt5::Widgets   Qt5::Core)
