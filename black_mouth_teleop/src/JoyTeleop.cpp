#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "black_mouth_teleop/JoyTeleop.hpp"
#include "black_mouth_kinematics/msg/all_leg_points.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;

JoyTeleop::JoyTeleop() : Node("joy_teleop_node")
{
  RCLCPP_INFO(this->get_logger(), "Joy Body IK Node initialized");

  _state.state = black_mouth_teleop::msg::TeleopState::INIT;

  _ik_publisher = this->create_publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);
  _default_pose_publisher = this->create_publisher<std_msgs::msg::Empty>("cmd_default_pose", 10);
  _vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, 
                          std::bind(&JoyTeleop::joyCallback, this, _1));

  _set_state_client = this->create_client<black_mouth_teleop::srv::SetTeleopState>("set_teleop_state");
  _set_body_control_publish_ik_client = this->create_client<std_srvs::srv::SetBool>("set_body_control_publish_ik");
  _reset_body_control_pid_client = this->create_client<std_srvs::srv::Empty>("reset_body_control_pid");

  _set_hw_state_client = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>("controller_manager/set_hardware_component_state");
  _switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");

  _ik_timer = this->create_wall_timer(50ms,  std::bind(&JoyTeleop::publishIK, this));
  _vel_timer = this->create_wall_timer(200ms, std::bind(&JoyTeleop::publishVel, this));
  _default_pose_timer = this->create_wall_timer(30ms, std::bind(&JoyTeleop::publishDefaultPose, this));

  _ik_timer->cancel();
  _vel_timer->cancel();
  _default_pose_timer->cancel();

  _resting = false;

  _ik_msg.body_leg_ik_trajectory.resize(1);
  _ik_msg.time_from_start.resize(1);

  _default_max_vel_map      = { {"lin_x", 0.05}, {"lin_y", 0.05}, {"ang_z", 0.5} };
  _default_axis_linear_map  = { {"x", 0},     {"y", 1},     {"z", 2} };
  _default_axis_angular_map = { {"roll", 3},  {"pitch", 4}, {"yaw", 5},
                                {"roll_inc", 6}, {"roll_dec", 7},
                                {"pitch_inc", 6}, {"pitch_dec", 7} };

  this->declare_parameters("max_vel", _default_max_vel_map);
  this->declare_parameters("axis_linear", _default_axis_linear_map);
  this->declare_parameters("axis_angular", _default_axis_angular_map);
  this->declare_parameter("joy_type", "generic");
  this->declare_parameter("lock", 0);
  this->declare_parameter("rest", 1);
  this->declare_parameter("body", 2);
  this->declare_parameter("walk", 3);
  this->declare_parameter("restart", 9);
  this->declare_parameter("filter_alpha", 0.0);

  this->get_parameters("max_vel", _max_vel_map);
  this->get_parameters("axis_linear", _axis_linear_map);
  this->get_parameters("axis_angular", _axis_angular_map);
  this->get_parameter("joy_type", _joy_type);
  this->get_parameter("lock", _lock_button);
  this->get_parameter("rest", _rest_button);
  this->get_parameter("body", _body_button);
  this->get_parameter("walk", _walk_button);
  this->get_parameter("restart", _restart_button);
  this->get_parameter("filter_alpha", _filter_alpha);

  _use_filter = _filter_alpha > 0.0;

  RCLCPP_INFO(this->get_logger(), "Using %s joystick", _joy_type.c_str());
  if (_filter_alpha)
    RCLCPP_INFO(this->get_logger(), "Filter alpha set to %.2f", _filter_alpha);

  _body_position_x_filter.setFilterAlpha(_filter_alpha);
  _body_position_y_filter.setFilterAlpha(_filter_alpha);
  _body_position_z_filter.setFilterAlpha(_filter_alpha);
  _body_rotation_x_filter.setFilterAlpha(_filter_alpha);
  _body_rotation_y_filter.setFilterAlpha(_filter_alpha);
  _body_rotation_z_filter.setFilterAlpha(_filter_alpha);

  while(!_set_state_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting.",
        _set_state_client->get_service_name());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s service not available, waiting again...",
      _set_state_client->get_service_name());
  }

  while(!_set_body_control_publish_ik_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting.",
        _set_body_control_publish_ik_client->get_service_name());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s service not available, waiting again...",
      _set_body_control_publish_ik_client->get_service_name());
  }

  while(!_reset_body_control_pid_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting.",
        _reset_body_control_pid_client->get_service_name());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s service not available, waiting again...",
      _reset_body_control_pid_client->get_service_name());
  }

  while(!_set_hw_state_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting.", 
      _set_hw_state_client->get_service_name());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s service not available, waiting again...",
      _set_hw_state_client->get_service_name());
  }

  while(!_switch_controller_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting.", 
      _switch_controller_client->get_service_name());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s service not available, waiting again...",
      _switch_controller_client->get_service_name());
  }

  auto request = std::make_shared<black_mouth_teleop::srv::SetTeleopState::Request>();
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
    _resting = false;
    auto request = std::make_shared<black_mouth_teleop::srv::SetTeleopState::Request>();
    request->state = _state;
    _set_state_client->async_send_request(request);
  }

  if (_state.state == black_mouth_teleop::msg::TeleopState::INIT)
    this->initState();
  else if (_state.state == black_mouth_teleop::msg::TeleopState::RESTING)
    this->restingState();
  else if (_state.state == black_mouth_teleop::msg::TeleopState::BODY_LOCKED)
    this->bodyLockedState();
  else if (_state.state == black_mouth_teleop::msg::TeleopState::CONTROLLING_BODY)
    this->controllingBodyState();
  else if (_state.state == black_mouth_teleop::msg::TeleopState::MOVING_BODY)
    this->movingBodyState(msg);
  else if (_state.state == black_mouth_teleop::msg::TeleopState::WALKING)
    this->walkingState(msg);
  else
    RCLCPP_ERROR(this->get_logger(), "Invalid Teleop State");

}


bool JoyTeleop::stateTransition(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto last_state = _state.state;

  if (msg->buttons[_rest_button])
    _resting = false;

  if (_state.state == black_mouth_teleop::msg::TeleopState::INIT)
  {
    if (msg->buttons[_rest_button]) {
      _state.state = black_mouth_teleop::msg::TeleopState::RESTING;

      _ik_timer->cancel();
      _vel_timer->cancel();
      _default_pose_timer->reset();

      // Activate hardware interface
      auto set_hw_state_request = controller_manager_msgs::srv::SetHardwareComponentState::Request();
      set_hw_state_request.name = "BlackMouthSystem";
      set_hw_state_request.target_state.id = 3; // 3 = Active

      RCLCPP_INFO(this->get_logger(), "Activating hardware_interface...");
      auto result_hw_state_goal = _set_hw_state_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>(set_hw_state_request));
      result_hw_state_goal.wait_for(1s);

      // Activate leg controllers
      auto switch_controller_request = controller_manager_msgs::srv::SwitchController::Request();
      switch_controller_request.activate_asap = true;
      switch_controller_request.strictness = 1; // BEST EFFORT
      switch_controller_request.activate_controllers.push_back("front_left_joint_trajectory_controller");
      switch_controller_request.activate_controllers.push_back("front_right_joint_trajectory_controller");
      switch_controller_request.activate_controllers.push_back("back_left_joint_trajectory_controller");
      switch_controller_request.activate_controllers.push_back("back_right_joint_trajectory_controller");

      RCLCPP_INFO(this->get_logger(), "Activating controllers...");
      auto result_switch_goal = _switch_controller_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>(switch_controller_request));
      result_switch_goal.wait_for(1s);

      _default_pose_publisher->publish(std_msgs::msg::Empty());

      // Delay
      RCLCPP_INFO(this->get_logger(), "Going to default position...");
      auto delay_time = std::chrono::system_clock::now() + 3s;
      while (std::chrono::system_clock::now() < delay_time){
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Ready!");
    }
  }

  else if (_state.state == black_mouth_teleop::msg::TeleopState::RESTING)
  {
    if (_resting)
    {
      _default_pose_timer->cancel();
      if (msg->buttons[_lock_button])
      {
        _state.state = black_mouth_teleop::msg::TeleopState::BODY_LOCKED;
      }
      else if (msg->buttons[_body_button])
      {
        _state.state = black_mouth_teleop::msg::TeleopState::CONTROLLING_BODY;
        
        _reset_body_control_pid_client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        _set_body_control_publish_ik_client->async_send_request(request);

      }
      else if (msg->buttons[_walk_button])
      {
        _state.state = black_mouth_teleop::msg::TeleopState::WALKING;
        _vel_timer->reset();
      }
      else if (msg->buttons[_restart_button]) {
        _state.state = black_mouth_teleop::msg::TeleopState::INIT;

        // Deactivate leg controllers
        auto switch_controller_request = controller_manager_msgs::srv::SwitchController::Request();
        switch_controller_request.strictness = 1; // BEST EFFORT
        switch_controller_request.deactivate_controllers.push_back("front_left_joint_trajectory_controller");
        switch_controller_request.deactivate_controllers.push_back("front_right_joint_trajectory_controller");
        switch_controller_request.deactivate_controllers.push_back("back_left_joint_trajectory_controller");
        switch_controller_request.deactivate_controllers.push_back("back_right_joint_trajectory_controller");

        auto result_switch_goal = _switch_controller_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>(switch_controller_request));
        result_switch_goal.wait_for(1s);

        // Deactivate hardware interface
        auto set_hw_state_request = controller_manager_msgs::srv::SetHardwareComponentState::Request();
        set_hw_state_request.name = "BlackMouthSystem";
        set_hw_state_request.target_state.id = 2; // Inactive 

        auto result_hw_state_goal = _set_hw_state_client->async_send_request(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>(set_hw_state_request));
        result_hw_state_goal.wait_for(1s);

      }
      else
      {
        _state.state = black_mouth_teleop::msg::TeleopState::MOVING_BODY;
        _ik_timer->reset();
      }
    }
    else _resting = true;
  }

  else if (_state.state == black_mouth_teleop::msg::TeleopState::CONTROLLING_BODY)
  {
    if (msg->buttons[_rest_button] || msg->buttons[_body_button])
    {
      _state.state = black_mouth_teleop::msg::TeleopState::RESTING;

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      _set_body_control_publish_ik_client->async_send_request(request);

      _default_pose_timer->reset();
    }
  }

  else if (_state.state == black_mouth_teleop::msg::TeleopState::MOVING_BODY)
  {
    if (msg->buttons[_rest_button])
    {
      _state.state = black_mouth_teleop::msg::TeleopState::RESTING;
      _ik_timer->cancel();
      _default_pose_timer->reset();
    }
    else if (msg->buttons[_lock_button])
    {
      _state.state = black_mouth_teleop::msg::TeleopState::BODY_LOCKED;
      _ik_timer->cancel();
    }
  }

  else if (_state.state == black_mouth_teleop::msg::TeleopState::BODY_LOCKED)
  {
    if (msg->buttons[_rest_button])
    {
      _state.state = black_mouth_teleop::msg::TeleopState::RESTING;
      _default_pose_timer->reset();
    }
    else if (msg->buttons[_lock_button] || msg->buttons[_body_button])
    {
      _state.state = black_mouth_teleop::msg::TeleopState::MOVING_BODY;
      _ik_timer->reset();
    }
  }

  else if (_state.state == black_mouth_teleop::msg::TeleopState::WALKING)
  {
    if (msg->buttons[_rest_button] || msg->buttons[_walk_button])
    {
      _state.state = black_mouth_teleop::msg::TeleopState::RESTING;
      _vel_timer->cancel();
      _default_pose_timer->reset();
    }
  }

  return (last_state != _state.state);
}


void JoyTeleop::initState()
{
  return;
}

void JoyTeleop::restingState()
{
  _ik_msg.body_leg_ik_trajectory.at(0).leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;

  _ik_msg.body_leg_ik_trajectory.at(0).body_position.x = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.y = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.z = 0.0;

  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y = 0.0;
  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z = 0.0;
}

void JoyTeleop::bodyLockedState()
{
  return;
}

void JoyTeleop::controllingBodyState()
{
  return;
}

void JoyTeleop::movingBodyState(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  _ik_msg.body_leg_ik_trajectory.at(0).leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;

  _ik_msg.body_leg_ik_trajectory.at(0).body_position.x = 0.05*msg->axes[_axis_linear_map["x"]];
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.y = 0.05*msg->axes[_axis_linear_map["y"]];
  _ik_msg.body_leg_ik_trajectory.at(0).body_position.z = 0.04*msg->axes[_axis_linear_map["z"]];

  _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z = 0.5*msg->axes[_axis_angular_map["yaw"]];
  
  if (_joy_type != "ps4")
  {
    if (msg->axes[_axis_angular_map["roll"]] == -1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x += 0.05;
    else if (msg->axes[_axis_angular_map["roll"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x -= 0.05;

    if (msg->axes[_axis_angular_map["pitch"]] == -1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y += 0.05;
    else if (msg->axes[_axis_angular_map["pitch"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y -= 0.05;
  }
  else
  {
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
  _vel_msg.linear.x = _max_vel_map["lin_x"]*msg->axes[_axis_linear_map["x"]];
  _vel_msg.linear.y = _max_vel_map["lin_y"]*msg->axes[_axis_linear_map["y"]];
  _vel_msg.angular.z = _max_vel_map["ang_z"]*msg->axes[_axis_angular_map["yaw"]];
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
  if (_use_filter)
  {
    this->filterIK();
    _ik_publisher->publish(_ik_msg_filtered);
  }
  else
    _ik_publisher->publish(_ik_msg);
}

void JoyTeleop::publishVel()
{
  _vel_publisher->publish(_vel_msg);
}

void JoyTeleop::publishDefaultPose()
{
  if (!_resting)
    _default_pose_publisher->publish(std_msgs::msg::Empty());
  if (_use_filter)
    this->filterIK();
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
