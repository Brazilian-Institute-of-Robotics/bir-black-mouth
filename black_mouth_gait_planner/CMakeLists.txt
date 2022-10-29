cmake_minimum_required(VERSION 3.5)
project(black_mouth_gait_planner)

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
set(dependencies
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rosidl_default_generators
  builtin_interfaces
  Eigen3
  black_mouth_kinematics
  black_mouth_control
  black_mouth_teleop
)

foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  # msg/...
  "srv/ComputeGaitTrajectory.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces black_mouth_kinematics
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(gait_traj_server src/compute_gait_trajectory.cpp)
add_executable(ik_gait_traj_server src/compute_ik_gait_trajectory.cpp)
ament_target_dependencies(gait_traj_server ${dependencies})
ament_target_dependencies(ik_gait_traj_server ${dependencies})

rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}" "rosidl_typesupport_cpp"
)
target_link_libraries(gait_traj_server "${cpp_typesupport_target}")
target_link_libraries(ik_gait_traj_server "${cpp_typesupport_target}")


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  gait_traj_server
  ik_gait_traj_server
  DESTINATION
  lib/${PROJECT_NAME}
)

# ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/trot_gait.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
