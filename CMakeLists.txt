cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif(POLICY CMP0048)
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
)

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

# include dir
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# ##################  Exec  ###################
## Hover ##
# set_mode
add_executable(set_mode src/set_mode.cpp)
target_link_libraries(set_mode ${catkin_LIBRARIES})

# set_command
add_executable(set_cmd src/set_command.cpp)
add_dependencies(set_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(set_cmd ${catkin_LIBRARIES})

# send_command
add_executable(send_cmd src/send_command.cpp)
add_dependencies(send_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(send_cmd ${catkin_LIBRARIES})

## Fix wing ##
# set_command
add_executable(set_cmd_fix src/set_command_fix.cpp)
add_dependencies(set_cmd_fix ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(set_cmd_fix ${catkin_LIBRARIES})

# recieve camera image
add_executable(recieve_video src/recieve_video.cpp)
add_dependencies(recieve_video ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(recieve_video ${catkin_LIBRARIES})

# External Cmd example
add_executable(ext_command src/ext_command.cpp)
add_dependencies(ext_command ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ext_command ${catkin_LIBRARIES})

