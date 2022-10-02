#ifndef BODY_ROTATION_CONTROL_HPP
#define BODY_ROTATION_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"

#include <memory>

class BodyControl : public rclcpp::Node
{
public:
  BodyControl();
  ~BodyControl();

private:
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void desiredRotationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  
  void publishBodyRotation();
  void publishIK();

  void computePID();

  rclcpp::TimerBase::SharedPtr _ik_timer;
  rclcpp::TimerBase::SharedPtr _body_timer;
  rclcpp::TimerBase::SharedPtr _pid_timer;

  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr _ik_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _body_rotation_publisher;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _desired_rotation_subscriber;

  geometry_msgs::msg::Vector3 _last_rotation_euler;
  geometry_msgs::msg::Vector3 _rotation_euler;

  geometry_msgs::msg::Vector3 _desired_body_rotation;
  geometry_msgs::msg::Vector3 _body_rotation_cmd;

  rclcpp::Time _current_time;
  rclcpp::Time _last_current_time;

  float _kp, _ki, _kd;
  float _error_roll, _error_pitch;
  float _sum_error_roll, _sum_error_pitch;
  float _last_error_roll, _last_error_pitch;

  bool _publish_ik;
};

#endif  // BODY_ROTATION_CONTROL_HPP
