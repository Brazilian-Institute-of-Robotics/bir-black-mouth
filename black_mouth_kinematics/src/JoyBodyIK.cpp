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

  _ik_publisher = this->create_publisher<black_mouth_kinematics::msg::BodyLegIK>("cmd_ik", 10);
  _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, 
                          std::bind(&JoyBodyIK::joyCallback, this, _1));

  _timer = this->create_wall_timer(50ms, std::bind(&JoyBodyIK::publishIK, this));

  _move_linear_x      = this->declare_parameter<int>("_move_linear_x", 1);
  _move_linear_y      = this->declare_parameter<int>("_move_linear_y", 0);
  _move_linear_z      = this->declare_parameter<int>("_move_linear_z", 2);
  _move_angular_yaw   = this->declare_parameter<int>("_move_angular_yaw", 3);
  _move_angular_roll  = this->declare_parameter<int>("_move_angular_roll", 4);
  _move_angular_pitch = this->declare_parameter<int>("_move_angular_pitch", 5);
}

JoyBodyIK::~JoyBodyIK()
{
}

void JoyBodyIK::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  _ik_msg.reference_link = black_mouth_kinematics::msg::BodyLegIK::FOOT_LINK_AS_REFERENCE;
  
  // TODO: Add more parameters and improve roll and pitch
  _ik_msg.body_position.x = 0.05*msg->axes[_move_linear_x];
  _ik_msg.body_position.y = 0.05*msg->axes[_move_linear_y];
  _ik_msg.body_position.z = 0.04*msg->axes[_move_linear_z];

  _ik_msg.body_rotation.x = 0.2*msg->axes[_move_angular_roll];
  _ik_msg.body_rotation.y = 0.2*msg->axes[_move_angular_pitch];
  _ik_msg.body_rotation.z = 0.5*msg->axes[_move_angular_yaw];

}

void JoyBodyIK::publishIK()
{
  _ik_publisher->publish(_ik_msg);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyBodyIK>());
  rclcpp::shutdown();
  return 0;
}
