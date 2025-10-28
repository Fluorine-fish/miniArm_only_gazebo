cmake_minimum_required(VERSION 3.8)
project(controller_with_ros)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
set(GZ_SIM_VER 8)
find_package(gz-sim${GZ_SIM_VER} REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(InvertedPendulumControllerPlugin SHARED src/*.cpp)
target_link_libraries(InvertedPendulumControllerPlugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
target_include_directories(InvertedPendulumControllerPlugin 
  PRIVATE include)
ament_target_dependencies(InvertedPendulumControllerPlugin
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS InvertedPendulumControllerPlugin
  DESTINATION lib
)

ament_package()
