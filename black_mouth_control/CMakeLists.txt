cmake_minimum_required(VERSION 3.5)
project(black_mouth_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# find dependencies
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(black_mouth_hw SHARED src/black_mouth_hardware_interface.cpp)
target_include_directories(black_mouth_hw PRIVATE include)
ament_target_dependencies(black_mouth_hw ${dependencies})

install(
  TARGETS
  black_mouth_hw
  DESTINATION 
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

## EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  black_mouth_hw
)

ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
