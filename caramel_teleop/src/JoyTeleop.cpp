#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp" // <--- NOVO
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "caramel_teleop/JoyTeleop.hpp"
#include "caramel_kinematics/msg/all_leg_points.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;

JoyTeleop::JoyTeleop() : Node("joy_teleop_node")
{
  RCLCPP_INFO(this->get_logger(), "Joy Body IK Node initialized");

  _state.state = caramel_teleop::msg::TeleopState::INIT;

  _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _sub_options.callback_group = _callback_group;

  _ik_publisher = this->create_publisher<caramel_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);
  _default_pose_publisher = this->create_publisher<std_msgs::msg::Empty>("cmd_default_pose", 10);
  _vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  _cpg_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_cpg", 10);
  
  // <--- NOVO: Inicializa o publicador do semáforo
  _cpg_activation_publisher = this->create_publisher<std_msgs::msg::Bool>("/robot_mode/cpg_active", 10);

  _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, 
                          std::bind(&JoyTeleop::joyCallback, this, _1), _sub_options);

  _set_state_client = this->create_client<caramel_teleop::srv::SetTeleopState>("set_teleop_state");
  _set_body_control_publish_ik_client = this->create_client<std_srvs::srv::SetBool>("set_body_control_publish_ik");
  _reset_body_control_pid_client = this->create_client<std_srvs::srv::Empty>("reset_body_control_pid");

  _set_hw_state_client = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>("controller_manager/set_hardware_component_state");
  _switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");

  _gait_parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "trot_gait_node",
                                                                                rmw_qos_profile_services_default, 
                                                                                _callback_group);

  _ik_timer = this->create_wall_timer(50ms,  std::bind(&JoyTeleop::publishIK, this));
  _vel_timer = this->create_wall_timer(200ms, std::bind(&JoyTeleop::publishVel, this));
  _default_pose_timer = this->create_wall_timer(5s, std::bind(&JoyTeleop::publishDefaultPose, this));

  _ik_timer->cancel();
  _vel_timer->cancel();
  _default_pose_timer->cancel();

  _ik_msg.body_leg_ik_trajectory.resize(1);
  _ik_msg.time_from_start.resize(1);

  _default_max_vel_map      = { {"lin_x", 0.10}, {"lin_y", 0.10}, {"ang_z", 0.5} };
  _default_gait_range_map   = { {"max_height", 0.05}, {"min_height", 0.025}, 
                                {"max_period", 1.0}, {"min_period", 0.25} };

  _default_axis_linear_map  = { {"x", 0},     {"y", 1},     {"z", 2} };
  _default_axis_angular_map = { {"roll", 3},  {"pitch", 4}, {"yaw", 5},
                                {"roll_inc", 6}, {"roll_dec", 7},
                                {"pitch_inc", 6}, {"pitch_dec", 7} };
  _default_gait_params_map  = { {"gait_height_inc", 8},  {"gait_height_dec", 9},
                                {"gait_period_inc", 10}, {"gait_period_dec", 11} };

  this->declare_parameters("max_vel", _default_max_vel_map);
  this->declare_parameters("gait_range", _default_gait_range_map);
  this->declare_parameters("axis_linear", _default_axis_linear_map);
  this->declare_parameters("axis_angular", _default_axis_angular_map);
  this->declare_parameters("gait_params", _default_gait_params_map);
  this->declare_parameter("joy_type", "generic");
  this->declare_parameter("lock", 0);
  this->declare_parameter("move", 1);
  this->declare_parameter("body", 2);
  this->declare_parameter("walk", 3);
  this->declare_parameter("restart", 9);
  this->declare_parameter("filter_alpha", 0.0);
  this->declare_parameter("cpg_toggle", 10);

  this->get_parameters("max_vel", _max_vel_map);
  this->get_parameters("gait_range", _gait_range_map);
  this->get_parameters("axis_linear", _axis_linear_map);
  this->get_parameters("axis_angular", _axis_angular_map);
  this->get_parameters("gait_params", _gait_params_map);
  this->get_parameter("joy_type", _joy_type);
  this->get_parameter("lock", _lock_button);
  this->get_parameter("move", _move_button);
  this->get_parameter("body", _body_button);
  this->get_parameter("walk", _walk_button);
  this->get_parameter("restart", _restart_button);
  this->get_parameter("filter_alpha", _filter_alpha);
  this->get_parameter("cpg_toggle", _cpg_toggle_button);

  _use_filter = _filter_alpha > 0.0;
  _max_vel_multiplier = 1.0;

  RCLCPP_INFO(this->get_logger(), "Using %s joystick", _joy_type.c_str());
  if (_filter_alpha)
    RCLCPP_INFO(this->get_logger(), "Filter alpha set to %.2f", _filter_alpha);

  _body_position_x_filter.setFilterAlpha(_filter_alpha);
  _body_position_y_filter.setFilterAlpha(_filter_alpha);
  _body_position_z_filter.setFilterAlpha(_filter_alpha);
  _body_rotation_x_filter.setFilterAlpha(_filter_alpha);
  _body_rotation_y_filter.setFilterAlpha(_filter_alpha);
  _body_rotation_z_filter.setFilterAlpha(_filter_alpha);

  // Checks se servicos estao disponiveis (mantive seu codigo original)
  while(!_set_state_client->wait_for_service(1s)){ if(!rclcpp::ok()) return; RCLCPP_INFO(this->get_logger(), "Waiting for set_state..."); }
  while(!_set_body_control_publish_ik_client->wait_for_service(1s)){ if(!rclcpp::ok()) return; RCLCPP_INFO(this->get_logger(), "Waiting for body_control..."); }
  while(!_reset_body_control_pid_client->wait_for_service(1s)){ if(!rclcpp::ok()) return; RCLCPP_INFO(this->get_logger(), "Waiting for reset_body_pid..."); }
  while(!_gait_parameters_client->wait_for_service(1s)){ if(!rclcpp::ok()) return; RCLCPP_INFO(this->get_logger(), "Waiting for gait_params..."); }
  while(!_set_hw_state_client->wait_for_service(1s)){ if(!rclcpp::ok()) return; RCLCPP_INFO(this->get_logger(), "Waiting for hw_state..."); }
  while(!_switch_controller_client->wait_for_service(1s)){ if(!rclcpp::ok()) return; RCLCPP_INFO(this->get_logger(), "Waiting for switch_controller..."); }

  auto request = std::make_shared<caramel_teleop::srv::SetTeleopState::Request>();
  request->state = _state;
  _set_state_client->async_send_request(request);
}

JoyTeleop::~JoyTeleop()
{
}

void JoyTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (this->stateTransition(msg))
  {
    auto request = std::make_shared<caramel_teleop::srv::SetTeleopState::Request>();
    request->state = _state;
    _set_state_client->async_send_request(request);
  }

  if (_state.state == caramel_teleop::msg::TeleopState::INIT)
    this->initState();
  else if (_state.state == caramel_teleop::msg::TeleopState::RESTING)
    this->restingState();
  else if (_state.state == caramel_teleop::msg::TeleopState::BODY_LOCKED)
    this->bodyLockedState();
  else if (_state.state == caramel_teleop::msg::TeleopState::CONTROLLING_BODY)
    this->controllingBodyState();
  else if (_state.state == caramel_teleop::msg::TeleopState::MOVING_BODY)
    this->movingBodyState(msg);
  else if (_state.state == caramel_teleop::msg::TeleopState::WALKING)
    this->walkingState(msg);
  else if (_state.state == caramel_teleop::msg::TeleopState::CPG_WALKING)
    this->cpgWalkingState(msg);
  else
    RCLCPP_ERROR(this->get_logger(), "Invalid Teleop State");
}


bool JoyTeleop::stateTransition(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto last_state = _state.state;

  // --- Lógica de detecção do Botão Toggle (R3 / Botão 10) ---
  static bool toggle_pressed_last_frame = false;
  bool toggle_just_pressed = false;

  if (msg->buttons.size() > (size_t)_cpg_toggle_button) {
      bool toggle_currently_pressed = (msg->buttons[_cpg_toggle_button] == 1);
      if (toggle_currently_pressed && !toggle_pressed_last_frame) {
          toggle_just_pressed = true;
      }
      toggle_pressed_last_frame = toggle_currently_pressed;
  } else {
      toggle_pressed_last_frame = false;
  }

  // Objeto para mensagem de ativação do CPG
  std_msgs::msg::Bool activation_msg;

  switch (_state.state)
  {
  case caramel_teleop::msg::TeleopState::INIT:
    if (msg->buttons[_lock_button] || msg->buttons[_move_button] || msg->buttons[_body_button] || msg->buttons[_walk_button] || msg->buttons[_restart_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::RESTING;

      _ik_timer->cancel();
      _vel_timer->cancel();

      // Garante que CPG começa desligado ao sair do INIT
      activation_msg.data = false;
      _cpg_activation_publisher->publish(activation_msg);

      // Activate hardware interface
      auto set_hw_state_request = controller_manager_msgs::srv::SetHardwareComponentState::Request();
      set_hw_state_request.name = "CaramelSystem";
      set_hw_state_request.target_state.id = 3; // 3 = Active
      _set_hw_state_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>(set_hw_state_request));

      // Activate leg controllers
      auto switch_controller_request = controller_manager_msgs::srv::SwitchController::Request();
      switch_controller_request.activate_asap = true;
      switch_controller_request.strictness = 1; // BEST EFFORT
      switch_controller_request.activate_controllers.push_back("front_left_joint_trajectory_controller");
      switch_controller_request.activate_controllers.push_back("front_right_joint_trajectory_controller");
      switch_controller_request.activate_controllers.push_back("back_left_joint_trajectory_controller");
      switch_controller_request.activate_controllers.push_back("back_right_joint_trajectory_controller");
      _switch_controller_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>(switch_controller_request));

      _default_pose_publisher->publish(std_msgs::msg::Empty());

      RCLCPP_INFO(this->get_logger(), "Going to default position...");
      auto delay_time = std::chrono::system_clock::now() + 3s;
      while (std::chrono::system_clock::now() < delay_time){ continue; }
      RCLCPP_INFO(this->get_logger(), "Ready!");
    }
    break;
  
  case caramel_teleop::msg::TeleopState::RESTING:
    if (msg->buttons[_lock_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::BODY_LOCKED;
      _default_pose_timer->cancel();
    }
    else if (msg->buttons[_body_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::CONTROLLING_BODY;
      _default_pose_timer->cancel();
      _reset_body_control_pid_client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      _set_body_control_publish_ik_client->async_send_request(request);
    }
    else if (msg->buttons[_walk_button])
    {
      // --- INICIANDO CPG COMO PADRÃO AO APERTAR WALK ---
      _state.state = caramel_teleop::msg::TeleopState::CPG_WALKING; 
      RCLCPP_INFO(this->get_logger(), "Iniciando modo CPG_WALKING");
      
      // Ativa o CPG
      activation_msg.data = true;
      _cpg_activation_publisher->publish(activation_msg);
          
      auto parameters = _gait_parameters_client->get_parameters({"gait_period"});
      std::future_status status = parameters.wait_for(50ms);
      if (status == std::future_status::ready) {
        double gait_period = parameters.get().at(0).as_double();
        _max_vel_multiplier = _gait_range_map["min_period"]/gait_period;
      } else {
        _max_vel_multiplier = 1.0;
      }
      _default_pose_timer->cancel();
      _vel_timer->cancel(); 
    }
    else if (msg->buttons[_restart_button]) {
      _state.state = caramel_teleop::msg::TeleopState::INIT;
      _default_pose_timer->cancel();

      // Deactivate controllers & HW (seu codigo original)
      auto switch_controller_request = controller_manager_msgs::srv::SwitchController::Request();
      switch_controller_request.strictness = 1; 
      switch_controller_request.deactivate_controllers.push_back("front_left_joint_trajectory_controller");
      switch_controller_request.deactivate_controllers.push_back("front_right_joint_trajectory_controller");
      switch_controller_request.deactivate_controllers.push_back("back_left_joint_trajectory_controller");
      switch_controller_request.deactivate_controllers.push_back("back_right_joint_trajectory_controller");
      _switch_controller_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>(switch_controller_request));

      auto set_hw_state_request = controller_manager_msgs::srv::SetHardwareComponentState::Request();
      set_hw_state_request.name = "CaramelSystem";
      set_hw_state_request.target_state.id = 2; 
      _set_hw_state_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>(set_hw_state_request));
    }
    else if (msg->buttons[_move_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::MOVING_BODY;
      _default_pose_timer->cancel();
      _ik_timer->reset();
    }
    break;

  case caramel_teleop::msg::TeleopState::CONTROLLING_BODY:
    if (msg->buttons[_body_button] || msg->buttons[_restart_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::RESTING;
      _default_pose_publisher->publish(std_msgs::msg::Empty());
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      _set_body_control_publish_ik_client->async_send_request(request);
    }
    break;

  case caramel_teleop::msg::TeleopState::MOVING_BODY:
    if (msg->buttons[_lock_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::BODY_LOCKED;
      _ik_timer->cancel();
    }
    else if (msg->buttons[_move_button] || msg->buttons[_restart_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::RESTING;
      _default_pose_publisher->publish(std_msgs::msg::Empty());
      _ik_timer->cancel();
    }
    break;

  case caramel_teleop::msg::TeleopState::BODY_LOCKED:
    if (msg->buttons[_move_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::MOVING_BODY;
      _ik_timer->reset();
    }
    else if (msg->buttons[_lock_button] || msg->buttons[_restart_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::RESTING;
      _default_pose_publisher->publish(std_msgs::msg::Empty());
    }
    break;

  case caramel_teleop::msg::TeleopState::WALKING: // MODO PID
    if (msg->buttons[_walk_button] || msg->buttons[_restart_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::RESTING;
      _default_pose_publisher->publish(std_msgs::msg::Empty());
      _vel_timer->cancel();

      // Garante que CPG dorme
      activation_msg.data = false;
      _cpg_activation_publisher->publish(activation_msg);
    }
    else if (toggle_just_pressed) // <--- APERTOU O BOTAO DE TROCA
    {
      RCLCPP_INFO(this->get_logger(), "Alternando para modo CPG_WALKING");
      _state.state = caramel_teleop::msg::TeleopState::CPG_WALKING;
      _vel_timer->cancel(); // Para o timer do PID
      
      // ACORDA O CPG
      activation_msg.data = true;
      _cpg_activation_publisher->publish(activation_msg);
    }
    break;

  case caramel_teleop::msg::TeleopState::CPG_WALKING: // MODO CPG
    if (msg->buttons[_walk_button] || msg->buttons[_restart_button])
    {
      _state.state = caramel_teleop::msg::TeleopState::RESTING;
      _default_pose_publisher->publish(std_msgs::msg::Empty());
      
      // Para o robô
      auto stop_msg = std::make_unique<geometry_msgs::msg::Twist>();
      _cpg_vel_publisher->publish(std::move(stop_msg));

      // MANDA O CPG DORMIR
      activation_msg.data = false;
      _cpg_activation_publisher->publish(activation_msg);
    }
    else if (toggle_just_pressed) // <--- APERTOU O BOTAO DE TROCA DE NOVO
    {
      RCLCPP_INFO(this->get_logger(), "Alternando para modo WALKING (PID)");
      _state.state = caramel_teleop::msg::TeleopState::WALKING;
      
      // Para o CPG e manda dormir
      auto stop_msg = std::make_unique<geometry_msgs::msg::Twist>();
      _cpg_vel_publisher->publish(std::move(stop_msg));
      
      activation_msg.data = false;
      _cpg_activation_publisher->publish(activation_msg);

      _vel_timer->reset(); // Liga o timer do PID
    }
    break;

  default:
    break;
  }

  return (last_state != _state.state);
}

void JoyTeleop::initState()
{
  return;
}

void JoyTeleop::restingState()
{
  _ik_msg.body_leg_ik_trajectory.at(0).leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.x = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.y = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.z = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z = 0.0;
}

void JoyTeleop::bodyLockedState() { return; }
void JoyTeleop::controllingBodyState() { return; }

void JoyTeleop::movingBodyState(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  _ik_msg.body_leg_ik_trajectory.at(0).leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.x = 0.05*msg->axes[_axis_linear_map["x"]];
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.y = 0.05*msg->axes[_axis_linear_map["y"]];
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.z = 0.04*msg->axes[_axis_linear_map["z"]];
  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z = 0.5*msg->axes[_axis_angular_map["yaw"]];
  
  // Logica dos botoes/eixos para roll/pitch (mantida original)
  if (_joy_type != "ps4") {
    if (msg->axes[_axis_angular_map["roll"]] == -1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x += 0.05;
    else if (msg->axes[_axis_angular_map["roll"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x -= 0.05;

    if (msg->axes[_axis_angular_map["pitch"]] == -1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y += 0.05;
    else if (msg->axes[_axis_angular_map["pitch"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y -= 0.05;
  } else {
    // Logica PS4 (mantida)
    if (msg->buttons[_axis_angular_map["roll_inc"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x += 0.05;
    if (msg->buttons[_axis_angular_map["roll_dec"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x -= 0.05;
    if (msg->buttons[_axis_angular_map["pitch_inc"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y += 0.05;
    if (msg->buttons[_axis_angular_map["pitch_dec"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y -= 0.05;
  }
}

void JoyTeleop::walkingState(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Ajuste de parametros do passo (mantido original)
  if (msg->buttons[_gait_params_map["gait_height_inc"]] || 
      msg->buttons[_gait_params_map["gait_height_dec"]] ||
      msg->buttons[_gait_params_map["gait_period_inc"]] ||
      msg->buttons[_gait_params_map["gait_period_dec"]] )
  {
    auto parameters = _gait_parameters_client->get_parameters({"gait_height", "gait_period"});
    std::future_status status = parameters.wait_for(10ms);
    if (status == std::future_status::ready) {
      double gait_height = parameters.get().at(0).as_double();
      double gait_period = parameters.get().at(1).as_double();
      
      if(msg->buttons[_gait_params_map["gait_height_inc"]]) {
        gait_height = std::min(_gait_range_map["max_height"], gait_height+0.005);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_height", gait_height)});
      } else if(msg->buttons[_gait_params_map["gait_height_dec"]]) {
        gait_height = std::max(_gait_range_map["min_height"], gait_height-0.005);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_height", gait_height)});
      }
      if(msg->buttons[_gait_params_map["gait_period_inc"]]) {
        gait_period = std::min(_gait_range_map["max_period"], gait_period+0.25);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_period", gait_period)});
        _max_vel_multiplier = _gait_range_map["min_period"]/gait_period;
      } else if(msg->buttons[_gait_params_map["gait_period_dec"]]) {
        gait_period = std::max(_gait_range_map["min_period"], gait_period-0.25);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_period", gait_period)});
        _max_vel_multiplier = _gait_range_map["min_period"]/gait_period;
      }
    }
  }

  _vel_msg.linear.x = _max_vel_multiplier*_max_vel_map["lin_x"]*msg->axes[_axis_linear_map["x"]];
  _vel_msg.linear.y = _max_vel_multiplier*_max_vel_map["lin_y"]*msg->axes[_axis_linear_map["y"]];
  _vel_msg.angular.z = _max_vel_multiplier*_max_vel_map["ang_z"]*msg->axes[_axis_angular_map["yaw"]];
}

void JoyTeleop::cpgWalkingState(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Mesma logica de ajuste de parametros para CPG
  if (msg->buttons[_gait_params_map["gait_height_inc"]] || 
      msg->buttons[_gait_params_map["gait_height_dec"]] ||
      msg->buttons[_gait_params_map["gait_period_inc"]] ||
      msg->buttons[_gait_params_map["gait_period_dec"]] )
  {
    auto parameters = _gait_parameters_client->get_parameters({"gait_height", "gait_period"});
    std::future_status status = parameters.wait_for(10ms);
    if (status == std::future_status::ready) {
      double gait_height = parameters.get().at(0).as_double();
      double gait_period = parameters.get().at(1).as_double();
      
      if(msg->buttons[_gait_params_map["gait_height_inc"]]) {
        gait_height = std::min(_gait_range_map["max_height"], gait_height+0.005);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_height", gait_height)});
      } else if(msg->buttons[_gait_params_map["gait_height_dec"]]) {
        gait_height = std::max(_gait_range_map["min_height"], gait_height-0.005);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_height", gait_height)});
      }
      if(msg->buttons[_gait_params_map["gait_period_inc"]]) {
        gait_period = std::min(_gait_range_map["max_period"], gait_period+0.25);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_period", gait_period)});
        _max_vel_multiplier = _gait_range_map["min_period"]/gait_period;
      } else if(msg->buttons[_gait_params_map["gait_period_dec"]]) {
        gait_period = std::max(_gait_range_map["min_period"], gait_period-0.25);
        _gait_parameters_client->set_parameters({rclcpp::Parameter("gait_period", gait_period)});
        _max_vel_multiplier = _gait_range_map["min_period"]/gait_period;
      }
    }
  }

  auto cpg_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
  cpg_vel_msg->linear.x = _max_vel_multiplier*_max_vel_map["lin_x"]*msg->axes[_axis_linear_map["x"]];
  cpg_vel_msg->linear.y = _max_vel_multiplier*_max_vel_map["lin_y"]*msg->axes[_axis_linear_map["y"]];
  cpg_vel_msg->angular.z = _max_vel_multiplier*_max_vel_map["ang_z"]*msg->axes[_axis_angular_map["yaw"]];

  _cpg_vel_publisher->publish(std::move(cpg_vel_msg));
}


void JoyTeleop::filterIK()
{
  _ik_msg_filtered = _ik_msg;
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_position.x = _body_position_x_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_position.x);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_position.y = _body_position_y_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_position.y);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_position.z = _body_position_z_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_position.z);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_rotation.x = _body_rotation_x_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_rotation.y = _body_rotation_y_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_rotation.z = _body_rotation_z_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z);
}

void JoyTeleop::publishIK()
{
  if (_use_filter) {
    this->filterIK();
    _ik_publisher->publish(_ik_msg_filtered);
  } else {
    _ik_publisher->publish(_ik_msg);
  }
}

void JoyTeleop::publishVel()
{
  _vel_publisher->publish(_vel_msg);
}

void JoyTeleop::publishDefaultPose()
{
  _default_pose_publisher->publish(std_msgs::msg::Empty());
  if (_use_filter) this->filterIK();
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<JoyTeleop>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}