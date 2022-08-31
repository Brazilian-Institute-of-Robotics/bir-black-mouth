#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "black_mouth_kinematics/JoyBodyIK.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;

JoyBodyIK::JoyBodyIK() : Node("joy_body_ik_node")
{
  RCLCPP_INFO(this->get_logger(), "Joy Body IK Node initialized");

  _ik_publisher = this->create_publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);
  _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, 
                          std::bind(&JoyBodyIK::joyCallback, this, _1));

  _timer = this->create_wall_timer(50ms, std::bind(&JoyBodyIK::publishIK, this));

  _locked = false;

  _ik_msg.body_leg_ik_trajectory.resize(1);
  _ik_msg.time_from_start.resize(1);

  _default_axis_linear_map  = { {"x", 0},    {"y", 1},     {"z", 2},   };
  _default_axis_angular_map = { {"roll", 3}, {"pitch", 4}, {"yaw", 5},
                                {"roll_inc", 6}, {"roll_dec", 7},
                                {"pitch_inc", 6}, {"pitch_dec", 7} };

  this->declare_parameters("axis_linear", _default_axis_linear_map);
  this->declare_parameters("axis_angular", _default_axis_angular_map);
  this->declare_parameter("joy_type", "generic");
  this->declare_parameter("lock", 0);
  this->declare_parameter("reset", 1);
  this->declare_parameter("filter_alpha", 0.0);

  this->get_parameters("axis_linear", _axis_linear_map);
  this->get_parameters("axis_angular", _axis_angular_map);
  this->get_parameter("joy_type", _joy_type);
  this->get_parameter("lock", _lock_button);
  this->get_parameter("reset", _reset_button);
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
}

JoyBodyIK::~JoyBodyIK()
{
}

void JoyBodyIK::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  _ik_msg.body_leg_ik_trajectory.at(0).leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  
  if (msg->buttons[_lock_button])
  {
    _locked = !_locked;
    if (_locked)
      RCLCPP_INFO(this->get_logger(), "Body locked");
    else
      RCLCPP_INFO(this->get_logger(), "Body unlocked");
  }

  if (!_locked)
  {
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

  if (msg->buttons[_reset_button])
  {
    RCLCPP_INFO(this->get_logger(), "Reset body position");
    _ik_msg.body_leg_ik_trajectory.at(0).body_position.x = 0.0;
    _ik_msg.body_leg_ik_trajectory.at(0).body_position.y = 0.0;
    _ik_msg.body_leg_ik_trajectory.at(0).body_position.z = 0.0;

    _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x = 0.0;
    _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y = 0.0;
    _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z = 0.0;
  }

}

void JoyBodyIK::filterIK()
{
  _ik_msg_filtered = _ik_msg;
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_position.x = _body_position_x_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_position.x);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_position.y = _body_position_y_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_position.y);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_position.z = _body_position_z_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_position.z);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_rotation.x = _body_rotation_x_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_rotation.y = _body_rotation_y_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y);
  _ik_msg_filtered.body_leg_ik_trajectory.at(0).body_rotation.z = _body_rotation_z_filter.filterData(_ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z);
}

void JoyBodyIK::publishIK()
{
  if (_use_filter)
  {
    this->filterIK();
    _ik_publisher->publish(_ik_msg_filtered);
  }
  else
    _ik_publisher->publish(_ik_msg);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyBodyIK>());
  rclcpp::shutdown();
  return 0;
}
