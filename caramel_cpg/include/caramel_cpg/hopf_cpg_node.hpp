#pragma once

#include "rclcpp/rclcpp.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;
using IK_MSG = caramel_kinematics::msg::BodyLegIKTrajectory;
using TWIST_MSG = geometry_msgs::msg::Twist;

class HopfCPGNode : public rclcpp::Node
{
public:
  HopfCPGNode();

private:
  void timer_callback();
  void cmd_vel_callback(const TWIST_MSG::SharedPtr msg);

  rclcpp::Publisher<IK_MSG>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<TWIST_MSG>::SharedPtr sub_vel_;

  int num_osc_;
  Eigen::VectorXd r_;
  Eigen::VectorXd phi_;
  Eigen::MatrixXd coupling_;

  rclcpp::Time last_time_;
  TWIST_MSG last_cmd_vel_;

  double omega_stance_base_;
  double omega_swing_base_;
  double turn_scale_;
};