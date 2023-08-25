cmake_minimum_required(VERSION 3.11)
if(POLICY CMP0079)
    cmake_policy(SET CMP0079 NEW)
endif(POLICY CMP0079)
# project name version languages
project(px4_cmd VERSION 1.0.3 LANGUAGES CXX)

# set C++ standard
set(CMAKE_CXX_FLAGS "-std=c++11")

# find package
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
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
)

# third party lib
add_subdirectory(third_party/jsoncpp)
add_subdirectory(third_party/tinyxml2)
add_subdirectory(third_party/qcustomplot)

# message generation
add_message_files(
    DIRECTORY msg
    FILES
    Command.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
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
target_link_libraries(set_cmd ${catkin_LIBRARIES})

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
target_link_libraries(recieve_video ${catkin_LIBRARIES})

# External Cmd example
add_executable(ext_cmd src/main/ext_command.cpp)
add_dependencies(ext_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ext_cmd ${catkin_LIBRARIES})

##################  GUI  ###################
# Generator
add_executable(generator src/gui/generator.cpp)
target_link_libraries(generator Qt5::Widgets)
target_link_libraries(generator jsoncpp)
target_link_libraries(generator tinyxml2)

# Controller
add_executable(controller src/gui/generator.cpp)
target_link_libraries(controller Qt5::Widgets)
target_link_libraries(controller jsoncpp)
target_link_libraries(controller tinyxml2)

# Monitor
add_executable(monitor src/gui/monitor.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})
target_link_libraries(monitor Qt5::Widgets)
target_link_libraries(monitor jsoncpp)
target_link_libraries(monitor tinyxml2)
target_link_libraries(monitor qcustomplot)