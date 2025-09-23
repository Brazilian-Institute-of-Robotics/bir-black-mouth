cmake_minimum_required(VERSION 3.8)
project(caramel_cpg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependências (SEM EIGEN)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(caramel_kinematics REQUIRED)

# Adicionar o executável do nó CPG
add_executable(cpg_node src/cpg_node.cpp)

# Ligar as dependências do ament/ROS 2
ament_target_dependencies(cpg_node
  "rclcpp"
  "geometry_msgs"
  "caramel_kinematics"
)

# Instalar o executável
install(TARGETS
  cpg_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar o diretório de launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()