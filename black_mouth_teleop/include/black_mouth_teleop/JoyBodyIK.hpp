#ifndef JOY_BODY_IK_HPP
#define JOY_BODY_IK_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "black_mouth_teleop/msg/teleop_state.hpp"
#include "black_mouth_teleop/srv/set_teleop_state.hpp"
#include "black_mouth_teleop/EMAFilter.hpp"

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

  bool stateTransition(const sensor_msgs::msg::Joy::SharedPtr msg);
  void initState();
  void restingState();
  void bodyLockedState();
  void controllingBodyState(const sensor_msgs::msg::Joy::SharedPtr msg);
  void walkingState(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr _ik_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

  rclcpp::Client<black_mouth_teleop::srv::SetTeleopState>::SharedPtr _set_state_client;

  black_mouth_kinematics::msg::BodyLegIKTrajectory _ik_msg;
  black_mouth_kinematics::msg::BodyLegIKTrajectory _ik_msg_filtered;

  black_mouth_teleop::msg::TeleopState _state;

  EMAFilter _body_position_x_filter;
  EMAFilter _body_position_y_filter;
  EMAFilter _body_position_z_filter;
  EMAFilter _body_rotation_x_filter;
  EMAFilter _body_rotation_y_filter;
  EMAFilter _body_rotation_z_filter;

  std::map<std::string, uint8_t> _axis_linear_map;
  std::map<std::string, uint8_t> _axis_angular_map;

  std::map<std::string, uint8_t> _default_axis_linear_map;
  std::map<std::string, uint8_t> _default_axis_angular_map;

  std::string _joy_type;

  uint8_t _lock_button;
  uint8_t _rest_button;
  uint8_t _walk_button;

  bool _resting;
  
  bool _use_filter;
  double _filter_alpha;

};

#endif // JOY_BODY_IK_HPP
