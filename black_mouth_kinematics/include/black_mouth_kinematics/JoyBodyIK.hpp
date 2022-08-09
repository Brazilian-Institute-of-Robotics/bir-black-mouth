#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik.hpp"

#include <memory>

class JoyBodyIK : public rclcpp::Node
{
public:
  JoyBodyIK();
  ~JoyBodyIK();
  
private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publishIK();

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIK>::SharedPtr _ik_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

  black_mouth_kinematics::msg::BodyLegIK _ik_msg;

  double _move_linear_x;
  double _move_linear_y;
  double _move_linear_z;
  double _move_angular_yaw;
  int    _move_angular_roll;
  int    _move_angular_pitch;

};
