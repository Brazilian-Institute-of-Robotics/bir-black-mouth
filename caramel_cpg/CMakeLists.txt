cmake_minimum_required(VERSION 3.8)
project(caramel_cpg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(caramel_kinematics REQUIRED)

# --- Adicione estas duas linhas ---
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# --- Novo nó de trote ---
add_executable(trot_node src/trot.cpp)
ament_target_dependencies(trot_node
  rclcpp
  builtin_interfaces
  caramel_kinematics
)

# --- Instalação ---
install(TARGETS
  trot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instala o diretório 'launch' no local correto para que o ros2 launch o encontre
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
