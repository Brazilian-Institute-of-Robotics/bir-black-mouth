#include "rclcpp/rclcpp.hpp"
#include "black_mouth_control/BodyControl.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <memory>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;

BodyControl::BodyControl() : Node("body_control")
{
  RCLCPP_INFO(this->get_logger(), "Body Control Node initialized");
  
  _kp = 0.5;
  _ki = 1.5;
  _kd = 0.0;

  _sum_error_roll = 0.0;
  _sum_error_pitch = 0.0;

  _publish_ik = true;

  _ik_publisher = this->create_publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);
  _body_control_publisher = this->create_publisher<black_mouth_control::msg::BodyControl>("body_control", 10);

  _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu/out", 10,
                          std::bind(&BodyControl::IMUCallback, this, _1));
  _desired_rotation_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("body_desired_rotation", 10,
                                       std::bind(&BodyControl::desiredRotationCallback, this, _1));

  _pid_timer = this->create_wall_timer(20ms, std::bind(&BodyControl::computePID, this));
  if (_publish_ik)
    _ik_timer = this->create_wall_timer(20ms, std::bind(&BodyControl::publishIK, this));

  _last_time = this->now();
  _current_time = this->now();

}

BodyControl::~BodyControl()
{
}

void BodyControl::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  _last_rotation_euler = _rotation_euler;
  tf2::Quaternion q(msg->orientation.x, 
                    msg->orientation.y, 
                    msg->orientation.z, 
                    msg->orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(_rotation_euler.x, _rotation_euler.y, _rotation_euler.z);
}

void BodyControl::desiredRotationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  _desired_body_rotation = *msg;
}

void BodyControl::publishIK()
{
  auto ik_msg = black_mouth_kinematics::msg::BodyLegIKTrajectory();
  
  ik_msg.body_leg_ik_trajectory.resize(1);
  ik_msg.time_from_start.resize(1);
  ik_msg.body_leg_ik_trajectory.at(0).leg_points.reference_link = 1;
  ik_msg.body_leg_ik_trajectory.at(0).body_rotation = _body_control.body_rotation;

  _ik_publisher->publish(ik_msg);
}

void BodyControl::computePID()
{
  _current_time = this->now();
  
  float dt = (_current_time - _last_time).seconds();

  _error_roll = _desired_body_rotation.x - _rotation_euler.x;
  _error_pitch = _desired_body_rotation.y - _rotation_euler.y;

  _sum_error_roll  += std::abs(_ki*_sum_error_roll*dt)  < 0.35 ? _error_roll  : 0.0;
  _sum_error_pitch += std::abs(_ki*_sum_error_pitch*dt) < 0.35 ? _error_pitch : 0.0;

  float PID_roll  = _kp*_error_roll  + _ki*_sum_error_roll*dt  + _kd*(_error_roll-_last_error_roll)/dt;
  float PID_pitch = _kp*_error_pitch + _ki*_sum_error_pitch*dt + _kd*(_error_pitch-_last_error_pitch)/dt;

  PID_roll  = std::abs(PID_roll)  < 0.4 ? PID_roll  : 0.4*(std::abs(PID_roll)/PID_roll);
  PID_pitch = std::abs(PID_pitch) < 0.4 ? PID_pitch : 0.4*(std::abs(PID_pitch)/PID_pitch);

  _body_control.pid_roll.current_val = _rotation_euler.x;
  _body_control.pid_roll.setpoint    = _desired_body_rotation.x;
  _body_control.pid_roll.error             = _error_roll;
  _body_control.pid_roll.error_integrative = _sum_error_roll*dt;
  _body_control.pid_roll.error_derivative  = (_error_roll-_last_error_roll)/dt;
  _body_control.pid_roll.last_timestep = _last_time;
  _body_control.pid_roll.timestep      = _current_time;
  _body_control.pid_roll.output = PID_roll;

  _body_control.pid_pitch.current_val = _rotation_euler.y;
  _body_control.pid_pitch.setpoint    = _desired_body_rotation.y;
  _body_control.pid_pitch.error             = _error_pitch;
  _body_control.pid_pitch.error_integrative = _sum_error_pitch*dt;
  _body_control.pid_pitch.error_derivative  = (_error_pitch-_last_error_pitch)/dt;
  _body_control.pid_pitch.last_timestep = _last_time;
  _body_control.pid_pitch.timestep      = _current_time;
  _body_control.pid_pitch.output = PID_pitch;

  _body_control.body_rotation.x = PID_roll;
  _body_control.body_rotation.y = PID_pitch;
  _body_control.body_rotation.z = 0.0;

  _last_error_roll = _error_roll;
  _last_error_pitch = _error_pitch;
  _last_time = _current_time;

  _body_control_publisher->publish(_body_control);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyControl>());
  rclcpp::shutdown();
  return 0;
}
