cmake_minimum_required(VERSION 3.11)
 
# 指定C++版本 1.9.5版本的jsoncpp需要用C++11版本的
 
set(CMAKE_CXX_STANDARD 11)
 
project(qcustomplot)

find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)

link_libraries(Qt5::Widgets)
 
add_library(${PROJECT_NAME} 
    STATIC
    qcustomplot.h
    qcustomplot.cpp
)
    
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})