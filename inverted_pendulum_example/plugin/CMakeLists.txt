cmake_minimum_required(VERSION 3.10)
project(inverted_pendulum_controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(gz-cmake3 REQUIRED)
set(GZ_SIM_VER 8)

gz_find_package(gz-sim${GZ_SIM_VER} REQUIRED COMPONENTS register)

#Add sources for plugin
add_library(InvertedPendulumControllerPlugin SHARED src/controller.cpp)
target_link_libraries(InvertedPendulumControllerPlugin
  PUBLIC gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)