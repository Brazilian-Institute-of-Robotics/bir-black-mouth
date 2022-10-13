#ifndef JOY_TELEOP_HPP
#define JOY_TELEOP_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "black_mouth_teleop/msg/teleop_state.hpp"
#include "black_mouth_teleop/srv/set_teleop_state.hpp"
#include "black_mouth_teleop/EMAFilter.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include <memory>

class JoyTeleop : public rclcpp::Node
{
public:
  JoyTeleop();
  ~JoyTeleop();
  
private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void filterIK();
  void publishIK();
  void publishVel();
  void publishDefaultPose();

  bool stateTransition(const sensor_msgs::msg::Joy::SharedPtr msg);
  void initState();
  void restingState();
  void bodyLockedState();
  void controllingBodyState();
  void movingBodyState(const sensor_msgs::msg::Joy::SharedPtr msg);
  void walkingState(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr _callback_group;
  rclcpp::SubscriptionOptions _sub_options;

  rclcpp::TimerBase::SharedPtr _ik_timer;
  rclcpp::TimerBase::SharedPtr _vel_timer;
  rclcpp::TimerBase::SharedPtr _default_pose_timer;

  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr _ik_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _default_pose_publisher;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

  rclcpp::Client<black_mouth_teleop::srv::SetTeleopState>::SharedPtr _set_state_client;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _set_body_control_publish_ik_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _reset_body_control_pid_client;

  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr _set_hw_state_client;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr _switch_controller_client;

  black_mouth_teleop::msg::TeleopState _state;

  black_mouth_kinematics::msg::BodyLegIKTrajectory _ik_msg;
  black_mouth_kinematics::msg::BodyLegIKTrajectory _ik_msg_filtered;

  geometry_msgs::msg::Twist _vel_msg;

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
  uint8_t _body_button;
  uint8_t _walk_button;
  uint8_t _restart_button;

  bool _resting;
  
  bool _use_filter;
  double _filter_alpha;

};

#endif // JOY_TELEOP_HPP
