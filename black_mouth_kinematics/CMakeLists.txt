cmake_minimum_required(VERSION 3.5)
project(black_mouth_kinematics)

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
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegJoints.msg"
  "msg/AllLegJoints.msg"
  "msg/AllLegPoints.msg"
  "msg/BodyLegIK.msg"
  "msg/BodyLegIKTrajectory.msg"
  "srv/InvKinematics.srv"
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(inv_kinematics_server src/compute_inverse_kinematics_server.cpp)
ament_target_dependencies(inv_kinematics_server rclcpp geometry_msgs tf2_eigen)

add_executable(inv_kinematics_client test/compute_inverse_kinematics_client.cpp)
ament_target_dependencies(inv_kinematics_client rclcpp)

add_executable(inv_kinematics_node src/InverseKinematics.cpp)
ament_target_dependencies(inv_kinematics_node rclcpp std_msgs trajectory_msgs)

add_executable(inv_kinematics_trajectory test/publish_ik_trajectory.cpp)
ament_target_dependencies(inv_kinematics_trajectory rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(inv_kinematics_server "${cpp_typesupport_target}")
target_link_libraries(inv_kinematics_client "${cpp_typesupport_target}")
target_link_libraries(inv_kinematics_node "${cpp_typesupport_target}")
target_link_libraries(inv_kinematics_trajectory "${cpp_typesupport_target}")

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

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
  
install(
  TARGETS
  inv_kinematics_server
  inv_kinematics_client
  inv_kinematics_node
  inv_kinematics_trajectory
  DESTINATION 
  lib/${PROJECT_NAME}
)
  
ament_package()