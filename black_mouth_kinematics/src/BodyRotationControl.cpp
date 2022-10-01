#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/BodyRotationControl.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <memory>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;

BodyRotationControl::BodyRotationControl() : Node("body_rotation_control")
{
  RCLCPP_INFO(this->get_logger(), "Body Rotation Control Node initialized");
  
  _kp = 1.0;
  _ki = 0.5;
  _kd = 0.0;

  _sum_error_roll = 0.0;
  _sum_error_pitch = 0.0;

  _publish_ik = true;

  _ik_publisher = this->create_publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);
  _body_rotation_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("desired_body_rotation", 10);

  _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10,
                          std::bind(&BodyRotationControl::IMUCallback, this, _1));
  
  _ik_timer = this->create_wall_timer(50ms, std::bind(&BodyRotationControl::publishIK, this));
  if (_publish_ik)
    _body_timer = this->create_wall_timer(50ms, std::bind(&BodyRotationControl::publishBodyRotation, this));
  _pid_timer = this->create_wall_timer(20ms, std::bind(&BodyRotationControl::computePID, this));

  _last_current_time = this->now();
  _current_time = this->now();

}

BodyRotationControl::~BodyRotationControl()
{
}

void BodyRotationControl::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  _last_rotation_euler = _rotation_euler;
  tf2::Quaternion q(msg->orientation.x, 
                    msg->orientation.y, 
                    msg->orientation.z, 
                    msg->orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(_rotation_euler.x, _rotation_euler.y, _rotation_euler.z);
}

void BodyRotationControl::publishBodyRotation()
{
  _body_rotation_publisher->publish(_body_rotation_cmd);
}

void BodyRotationControl::publishIK()
{
  auto ik_msg = black_mouth_kinematics::msg::BodyLegIKTrajectory();
  
  ik_msg.body_leg_ik_trajectory.resize(1);
  ik_msg.time_from_start.resize(1);
  ik_msg.body_leg_ik_trajectory.at(0).body_rotation = _body_rotation_cmd;

  _ik_publisher->publish(ik_msg);
}

void BodyRotationControl::computePID()
{
  _current_time = this->now();

  _error_roll = _desired_body_rotation.x - _rotation_euler.x;
  _error_pitch = _desired_body_rotation.y - _rotation_euler.y;

  _sum_error_roll += _error_roll;
  _sum_error_pitch += _error_pitch;

  float dt = (_current_time - _last_current_time).seconds();
  // RCLCPP_INFO(this->get_logger(), "Time: %s=.3f", dt);

  float PID_roll  = _kp*_error_roll  + _ki*_sum_error_roll*dt  + _kd*(_error_roll-_last_error_roll)/dt;
  float PID_pitch = _kp*_error_pitch + _ki*_sum_error_pitch*dt + _kd*(_error_pitch-_last_error_pitch)/dt;

  _body_rotation_cmd.x = PID_roll;
  _body_rotation_cmd.x = PID_pitch;

  _last_error_roll = _error_roll;
  _last_error_pitch = _error_pitch;
  _last_current_time = _current_time;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyRotationControl>());
  rclcpp::shutdown();
  return 0;
}
