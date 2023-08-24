cmake_minimum_required(VERSION 3.11)
 
# 指定C++版本 1.9.5版本的jsoncpp需要用C++11版本的
 
set(CMAKE_CXX_STANDARD 11)
 
project(qcustomplot)

find_package(Qt5 COMPONENTS 
    Widgets 
    Gui
    Core
    PrintSupport
    REQUIRED)

add_library( ${PROJECT_NAME} 
    STATIC
    qcustomplot.cpp
    qcustomplot.h)

set_target_properties( ${PROJECT_NAME} PROPERTIES AUTOMOC TRUE )
target_link_libraries( ${PROJECT_NAME} Qt5::Widgets Qt5::PrintSupport)
    
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})