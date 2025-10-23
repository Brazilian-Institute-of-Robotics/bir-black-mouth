#pragma once

#include "rclcpp/rclcpp.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;
using IK_MSG = caramel_kinematics::msg::BodyLegIKTrajectory;
using namespace Eigen;

class HopfCPGNode : public rclcpp::Node
{
public:
  HopfCPGNode();

private:
  void timer_callback();

  // === Membros ===
  rclcpp::Publisher<IK_MSG>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int num_osc_;
  VectorXd r_;
  VectorXd phi_;
  MatrixXd coupling_;

  rclcpp::Time last_time_;
};
