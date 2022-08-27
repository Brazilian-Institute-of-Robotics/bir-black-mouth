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

  _default_axis_linear_map  = { {"x", 1},    {"y", 0},     {"z", 2},   };
  _default_axis_angular_map = { {"roll", 4}, {"pitch", 5}, {"yaw", 3}, };

  this->declare_parameters("axis_linear", _default_axis_linear_map);
  this->declare_parameters("axis_angular", _default_axis_angular_map);
  this->declare_parameter("lock", 2);
  this->declare_parameter("reset", 3);

  this->get_parameters("axis_linear", _axis_linear_map);
  this->get_parameters("axis_angular", _axis_angular_map);
  this->get_parameter("lock", _lock_button);
  this->get_parameter("reset", _reset_button);

  // TODO: Get parameters for filters

  _body_position_x_filter.setFilterAlpha(0.9);
  _body_position_y_filter.setFilterAlpha(0.9);
  _body_position_z_filter.setFilterAlpha(0.9);
  _body_rotation_x_filter.setFilterAlpha(0.9);
  _body_rotation_y_filter.setFilterAlpha(0.9);
  _body_rotation_z_filter.setFilterAlpha(0.9);
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
      RCLCPP_INFO(rclcpp::get_logger("joy_body_ik"), "Body locked");
    else
      RCLCPP_INFO(rclcpp::get_logger("joy_body_ik"), "Body unlocked");
  }

  if (!_locked)
  {
    _ik_msg.body_leg_ik_trajectory.at(0).body_position.x = 0.05*msg->axes[_axis_linear_map["x"]];
    _ik_msg.body_leg_ik_trajectory.at(0).body_position.y = 0.05*msg->axes[_axis_linear_map["y"]];
    _ik_msg.body_leg_ik_trajectory.at(0).body_position.z = 0.04*msg->axes[_axis_linear_map["z"]];

    _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.z = 0.5*msg->axes[_axis_angular_map["yaw"]];

    if (msg->axes[_axis_angular_map["roll"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x += 0.05;
    else if (msg->axes[_axis_angular_map["roll"]] == -1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.x -= 0.05;

    if (msg->axes[_axis_angular_map["pitch"]] == 1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y < 0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y += 0.05;
    else if (msg->axes[_axis_angular_map["pitch"]] == -1 && _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y > -0.35)
      _ik_msg.body_leg_ik_trajectory.at(0).body_rotation.y -= 0.05;
  }

  if (msg->buttons[_reset_button])
  {
    RCLCPP_INFO(rclcpp::get_logger("joy_body_ik"), "Reset body position");
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
  this->filterIK();
  _ik_publisher->publish(_ik_msg_filtered);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyBodyIK>());
  rclcpp::shutdown();
  return 0;
}
