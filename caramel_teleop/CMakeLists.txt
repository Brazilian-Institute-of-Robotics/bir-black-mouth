cmake_minimum_required(VERSION 3.5)
project(caramel_teleop)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(caramel_kinematics REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(controller_manager REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeleopState.msg"
  "srv/SetTeleopState.srv"
  "srv/GetTeleopState.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(teleop_state_server src/teleop_state_server.cpp)
ament_target_dependencies(teleop_state_server rclcpp)

add_executable(joy_teleop src/JoyTeleop.cpp)
ament_target_dependencies(joy_teleop rclcpp std_srvs std_msgs sensor_msgs geometry_msgs caramel_kinematics controller_manager)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(teleop_state_server "${cpp_typesupport_target}")
target_link_libraries(joy_teleop "${cpp_typesupport_target}")

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

include_directories(include)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  teleop_state_server
  joy_teleop
  DESTINATION 
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
