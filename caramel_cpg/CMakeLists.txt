cmake_minimum_required(VERSION 3.8)
project(caramel_cpg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(caramel_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(hopf_cpg_node src/hopf_cpg_node.cpp)
ament_target_dependencies(hopf_cpg_node
  rclcpp
  builtin_interfaces
  caramel_kinematics
)

install(TARGETS
  hopf_cpg_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
