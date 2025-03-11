cmake_minimum_required(VERSION 3.5)
project(caramel_gazebo)

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

# Encontrar pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros2_control REQUIRED)  # Adicionado para a dependência do gazebo_ros2_control
find_package(ament_cmake REQUIRED)
find_package(ros_ign_gazebo REQUIRED)
find_package(tf2_ros REQUIRED)

# Instalar arquivos de launch e worlds
install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Adicionar dependências
ament_export_dependencies(gazebo_ros2_control)  # Exportando a dependência

# Testes de qualidade do código
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # a seguinte linha ignora o verificador de copyrights
  # descomente a linha quando não houver copyright e licença em todos os arquivos
  #set(ament_cmake_copyright_FOUND TRUE)
  # a seguinte linha ignora o cpplint (funciona apenas em repositórios git)
  # descomente a linha quando este pacote não estiver em um repositório git
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
