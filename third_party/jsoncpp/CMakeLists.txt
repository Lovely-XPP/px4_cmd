cmake_minimum_required(VERSION 3.11)
 
# 指定C++版本 1.9.5版本的jsoncpp需要用C++11版本的
 
set(CMAKE_CXX_STANDARD 11)
 
project(jsoncpp)
 
add_library(${PROJECT_NAME} 
    STATIC
    src/json_tool.h 
    src/json_valueiterator.inl 
    src/json_reader.cpp 
    src/json_value.cpp 
    src/json_writer.cpp)
    
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)