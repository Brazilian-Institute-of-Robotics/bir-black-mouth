#ifndef JOY_BODY_IK_HPP
#define JOY_BODY_IK_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "black_mouth_kinematics/EMAFilter.hpp"

#include <memory>

class JoyBodyIK : public rclcpp::Node
{
public:
  JoyBodyIK();
  ~JoyBodyIK();
  
private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void filterIK();
  void publishIK();

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr _ik_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

  black_mouth_kinematics::msg::BodyLegIKTrajectory _ik_msg;

  std::map<std::string, int8_t> _axis_linear_map;
  std::map<std::string, int8_t> _axis_angular_map;

  std::map<std::string, int8_t> _default_axis_linear_map;
  std::map<std::string, int8_t> _default_axis_angular_map;

  int8_t _lock_button;
  int8_t _reset_button;

  bool _locked;

};

#endif // JOY_BODY_IK_HPP
