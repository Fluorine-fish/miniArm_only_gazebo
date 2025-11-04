cmake_minimum_required(VERSION 3.8)
project(alg_calc)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Source files
set(ALG_CALC_SOURCES
  src/main.cpp
  src/matrix.cpp
  src/robotics.cpp
  src/utils.cpp
)

add_executable(alg_calc ${ALG_CALC_SOURCES})

add_executable(alg_calc_node src/calculator_node.cpp)
ament_target_dependencies(alg_calc_node rclcpp std_msgs controller)

# Link implementation sources needed by calculator_node
target_sources(alg_calc_node PRIVATE
  src/matrix.cpp
  src/robotics.cpp
  src/utils.cpp
)

target_include_directories(alg_calc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(alg_calc_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
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

ament_package()
