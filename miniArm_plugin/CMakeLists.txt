cmake_minimum_required(VERSION 3.10)
project(miniarm_position_control)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(gz-sim8 REQUIRED)

add_library(JointPositionControlPlugin SHARED src/joint_position_control.cpp)
target_link_libraries(JointPositionControlPlugin gz-sim8::gz-sim8)