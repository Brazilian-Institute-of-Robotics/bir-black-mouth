cmake_minimum_required(VERSION 3.8)
project(caramel_cpg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(caramel_kinematics REQUIRED)

add_executable(cpg_node src/cpg_node.cpp)
ament_target_dependencies(cpg_node
  rclcpp
  builtin_interfaces
  caramel_kinematics
)

install(TARGETS cpg_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
