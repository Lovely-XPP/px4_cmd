# Copyright (c) 2023 易鹏 中山大学航空航天学院
# Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

cmake_minimum_required(VERSION 3.11)

# project name version languages
project(px4_cmd VERSION 1.0.3 LANGUAGES CXX)

# set C++ standard
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find package
# opencv
find_package(OpenCV REQUIRED)
# catkin package
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    mavros_msgs
    geometry_msgs
    message_generation
    tf
)
find_package(Boost REQUIRED COMPONENTS system)

# Qt depend
find_package(Qt5 COMPONENTS 
    Widgets
    Gui
    Core)
set(CMAKE_AUTOMOC ON)
    
# include dir
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# third party lib
add_subdirectory(third_party/jsoncpp)
add_subdirectory(third_party/tinyxml2)
add_subdirectory(third_party/qcustomplot)

# install python module
catkin_python_setup()

# message generation
add_message_files(
    DIRECTORY msg
    FILES
    Command.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime std_msgs
)

###################  Exec  ###################
## Hover ##
# set_mode
add_executable(set_mode src/main/set_mode.cpp)
target_link_libraries(set_mode ${catkin_LIBRARIES})

# set_command
add_executable(set_cmd src/main/set_command.cpp)
add_dependencies(set_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(set_cmd ${catkin_LIBRARIES} ${OpenCV_LIBS})

# send_command
add_executable(send_cmd src/main/send_command.cpp)
add_dependencies(send_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(send_cmd ${catkin_LIBRARIES})

## Fix wing ##
# set_command
add_executable(set_cmd_fix src/main/set_command_fix.cpp)
add_dependencies(set_cmd_fix ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(set_cmd_fix ${catkin_LIBRARIES})

# recieve camera image
add_executable(recieve_video src/main/recieve_video.cpp)
add_dependencies(recieve_video ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(recieve_video ${catkin_LIBRARIES} ${OpenCV_LIBS})

##################  GUI  ###################
# Generator
link_libraries(Qt5::Widgets)
add_executable(generator src/gui/generator.cpp)
add_dependencies(generator ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(generator jsoncpp)
target_link_libraries(generator tinyxml2)

# Controller
add_executable(controller src/gui/controller.cpp)
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

# Monitor
add_executable(monitor src/gui/monitor.cpp)
add_dependencies(monitor ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(monitor ${catkin_LIBRARIES})
target_link_libraries(monitor qcustomplot)

################## External cmd  ##################
add_library(${PROJECT_NAME} src/template/vehicle_external_command.cpp src/template/single_vehicle_external_command.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
