cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME controller_plugin_base)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  rclcpp
  as2_core
  as2_msgs
  nav_msgs
  geometry_msgs
  trajectory_msgs
  yaml-cpp
  as2_control_command_handlers
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_test test/plugin_base_build_test.cpp)
ament_target_dependencies(${PROJECT_NAME}_test ${PROJECT_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
  # include/${PROJECT_NAME}
)

ament_package()
