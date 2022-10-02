#ifndef BODY_ROTATION_CONTROL_HPP
#define BODY_ROTATION_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "black_mouth_control/msg/body_control.hpp"

#include <memory>

class BodyControl : public rclcpp::Node
{
public:
  BodyControl();
  ~BodyControl();

private:
  void publishIK();
  void computePID();

  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void desiredRotationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

  void setPublishIK(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::TimerBase::SharedPtr _pid_timer;
  rclcpp::TimerBase::SharedPtr _ik_timer;

  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr _ik_publisher;
  rclcpp::Publisher<black_mouth_control::msg::BodyControl>::SharedPtr _body_control_publisher;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _desired_rotation_subscriber;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _set_publish_ik_service;

  geometry_msgs::msg::Vector3 _last_rotation_euler;
  geometry_msgs::msg::Vector3 _rotation_euler;

  geometry_msgs::msg::Vector3 _desired_body_rotation;
  black_mouth_control::msg::BodyControl _body_control;

  rclcpp::Time _current_time;
  rclcpp::Time _last_time;

  float _kp, _ki, _kd;
  float _error_roll, _error_pitch;
  float _sum_error_roll, _sum_error_pitch;
  float _last_error_roll, _last_error_pitch;

};

#endif  // BODY_ROTATION_CONTROL_HPP
