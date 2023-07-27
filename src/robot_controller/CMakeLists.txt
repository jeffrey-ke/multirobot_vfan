cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(custom_msgs REQUIRED)

#include
include_directories(${PROJECT_SOURCE_DIR}/include/robot_controller)

#libraries
  ## COntroller
  add_library(Controller src/Controller.cpp)
# compile ros nodes
  ## robot node
  add_executable(RobotNode src/RobotNode.cpp)
  ament_target_dependencies(RobotNode rclcpp std_msgs geometry_msgs turtlesim custom_msgs)
  target_link_libraries(RobotNode Controller) 
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

install(TARGETS
  RobotNode
  DESTINATION lib/${PROJECT_NAME})
# Install launch files.

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
