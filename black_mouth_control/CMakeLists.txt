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
  tf2
  geometry_msgs
  sensor_msgs
  std_srvs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  dynamixel_sdk
  black_mouth_kinematics
  rosidl_default_generators
)

# find dependencies
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PID.msg"
  "msg/BodyControl.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(${PROJECT_NAME}_hw 
  SHARED 
  src/black_mouth_hardware_interface.cpp
)

add_executable(body_control src/BodyControl.cpp)
ament_target_dependencies(body_control ${dependencies})

target_include_directories(${PROJECT_NAME}_hw PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_hw ${dependencies})

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(body_control "${cpp_typesupport_target}")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface black_mouth_control.xml)

install(
  TARGETS ${PROJECT_NAME}_hw
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS body_control
  DESTINATION lib/${PROJECT_NAME}
)

# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/test_trot.py

  DESTINATION lib/${PROJECT_NAME})

## EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}_hw
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
