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

}

JoyBodyIK::~JoyBodyIK()
{
}

void JoyBodyIK::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{

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
