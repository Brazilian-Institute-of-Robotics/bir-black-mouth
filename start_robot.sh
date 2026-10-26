#!/bin/bash

# 1. Carrega o ambiente do ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Carrega o seu Workspace
source /home/caramel/ros2_ws/install/setup.bash

# 3. (Opcional) Domain ID
# export ROS_DOMAIN_ID=42

# Funcao que aguarda o launch subir, envia o START e GARANTE as 4 pernas ativas.
auto_init() {
  # --- Espera ATIVA pelos controladores (nao "sleep" cego) ---
  echo "[AUTO_INIT] Aguardando os controladores carregarem..."
  for i in $(seq 1 150); do
    n=$(ros2 control list_controllers 2>/dev/null | grep -cE "joint_trajectory_controller|joint_state_broadcaster")
    if [ "${n:-0}" -ge 5 ]; then
      echo "[AUTO_INIT] ${n} controladores carregados."
      break
    fi
    sleep 1
  done

  echo "[AUTO_INIT] Aguardando o joy_teleop assinar /joy..."
  for i in $(seq 1 40); do
    if ros2 node list 2>/dev/null | grep -q joy_teleop_node; then
      echo "[AUTO_INIT] joy_teleop no ar."
      break
    fi
    sleep 1
  done
  sleep 2

  echo "[AUTO_INIT] Enviando comando de START (INIT -> RESTING)..."
  ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
  sleep 3

  # --- WATCHDOG DE ATIVACAO ---
  # Garante que as 4 pernas fiquem ATIVAS, nao importa quem disparou o START
  # (inclusive um aperto PRECOCE do joystick fisico, que ativava parcial e
  # deixava uma perna inativa = "corpo tremendo"/torto). Auto-cura a race.
  echo "[AUTO_INIT] Watchdog: garantindo as 4 pernas ativas..."
  for r in $(seq 1 20); do
    act=$(ros2 control list_controllers 2>/dev/null | grep "trajectory_controller" | grep -c "active")
    if [ "${act:-0}" -ge 4 ]; then
      echo "[AUTO_INIT] 4 pernas ativas."
      break
    fia
    for c in front_left front_right back_left back_right; do
      if ! ros2 control list_controllers 2>/dev/null | grep -q "${c}_joint_trajectory_controller active"; then
        echo "[AUTO_INIT]   ativando ${c}..."
        ros2 control set_controller_state "${c}_joint_trajectory_controller" active >/dev/null 2>&1
      fi
    done
    # Reenvia a pose default para alinhar pernas ativadas tardiamente
    ros2 topic pub --once /cmd_default_pose std_msgs/msg/Empty "{}" >/dev/null 2>&1
    sleep 1.5
  done

  echo "[AUTO_INIT] Sequencia concluida."
}

# 4. Dispara a rotina em background
auto_init &

# 5. Inicia o launch em foreground (mantem os logs visiveis na tela)
ros2 launch caramel_bringup bringup.launch.py
