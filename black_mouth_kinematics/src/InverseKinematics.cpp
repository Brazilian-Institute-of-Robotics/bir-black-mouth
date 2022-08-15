#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/InverseKinematics.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

InverseKinematics::InverseKinematics() : Node("inverse_kinematics_node")
{
  RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Node initialized");

  _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _sub_options.callback_group = _callback_group;

  _cmd_subscriber = this->create_subscription<black_mouth_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10, 
                          std::bind(&InverseKinematics::IKCallback, this, _1), _sub_options);

  _front_right_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("front_right_joint_trajectory_controller/joint_trajectory", 10);
  _front_left_trajectory_publisher  = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("front_left_joint_trajectory_controller/joint_trajectory", 10);
  _back_left_trajectory_publisher   = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("back_left_joint_trajectory_controller/joint_trajectory", 10);
  _back_right_trajectory_publisher  = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("back_right_joint_trajectory_controller/joint_trajectory", 10);

  _ik_client = this->create_client<black_mouth_kinematics::srv::InvKinematics>("compute_inverse_kinematics", 
                                                                                rmw_qos_profile_services_default, 
                                                                                _callback_group);

  _default_joint_limits = { {"hip_roll_limit", 0.785}, {"hip_pitch_limit", 0.785}, {"elbow_limit", 0.785} };
  this->declare_parameters("joint_limits", _default_joint_limits);
  this->get_parameters("joint_limits", _joint_limits);
  
  while(!_ik_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the IK service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IK service not available, waiting again...");
  }
}

InverseKinematics::~InverseKinematics()
{
}

void InverseKinematics::IKCallback(const black_mouth_kinematics::msg::BodyLegIKTrajectory::SharedPtr msg)
{
  // auto start = std::chrono::steady_clock::now();

  this->_cmd_ik_msg = *msg;

  this->computeIKAndPublishJoints();

  // auto end = std::chrono::steady_clock::now();
  // std::cout << "Elapsed time in microseconds: "
            // << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
            // << " µs" << std::endl;
}

void InverseKinematics::computeIKAndPublishJoints()
{
  this->computeIK();
  if (this->checkJointAngles())
    this->publishAllJoints();
}

void InverseKinematics::computeIK()
{
  auto request = std::make_shared<black_mouth_kinematics::srv::InvKinematics::Request>();

  _all_leg_joints.resize(_cmd_ik_msg.body_leg_ik_trajectory.size());
  
  for (int i=0; i<(int)_cmd_ik_msg.body_leg_ik_trajectory.size(); i++)
  {
    request->body_leg_ik = this->_cmd_ik_msg.body_leg_ik_trajectory.at(i);  
    auto result = this->_ik_client->async_send_request(request);
    std::future_status status = result.wait_for(1.5ms);
    
    if (status == std::future_status::ready)
      _all_leg_joints.at(i) = result.get()->leg_joints;
    else
      RCLCPP_WARN(rclcpp::get_logger("ik_client"), "Failed to comput leg joints, timeout");
  }
}

bool InverseKinematics::checkJointAngles()
{
  for (auto leg_joints : _all_leg_joints)
  {
    if (std::isnan(leg_joints.front_right_leg.hip_roll_joint) ||
        std::isnan(leg_joints.front_right_leg.hip_pitch_joint) ||
        std::isnan(leg_joints.front_right_leg.elbow_joint) ||
        std::isnan(leg_joints.front_left_leg.hip_roll_joint) ||
        std::isnan(leg_joints.front_left_leg.hip_pitch_joint) ||
        std::isnan(leg_joints.front_left_leg.elbow_joint) ||
        std::isnan(leg_joints.back_left_leg.hip_roll_joint) ||
        std::isnan(leg_joints.back_left_leg.hip_pitch_joint) ||
        std::isnan(leg_joints.back_left_leg.elbow_joint) ||
        std::isnan(leg_joints.back_right_leg.hip_roll_joint) ||
        std::isnan(leg_joints.back_right_leg.hip_pitch_joint) ||
        std::isnan(leg_joints.back_right_leg.elbow_joint))
    {
      RCLCPP_WARN(rclcpp::get_logger("ik_node"), "NaN value computed, impossible to set IK");
      return false;
    }

    if (abs(leg_joints.front_right_leg.hip_roll_joint) > _joint_limits["hip_roll_limit"] ||
        abs(leg_joints.front_left_leg.hip_roll_joint)  > _joint_limits["hip_roll_limit"] ||
        abs(leg_joints.back_left_leg.hip_roll_joint)   > _joint_limits["hip_roll_limit"] ||
        abs(leg_joints.back_right_leg.hip_roll_joint)  > _joint_limits["hip_roll_limit"])
    {
      RCLCPP_WARN(rclcpp::get_logger("ik_node"), "Hip roll joint angle out of range");
      return false;
    }
    if (abs(leg_joints.front_right_leg.hip_pitch_joint) > _joint_limits["hip_pitch_limit"] ||
        abs(leg_joints.front_left_leg.hip_pitch_joint)  > _joint_limits["hip_pitch_limit"] ||
        abs(leg_joints.back_left_leg.hip_pitch_joint)   > _joint_limits["hip_pitch_limit"] ||
        abs(leg_joints.back_right_leg.hip_pitch_joint)  > _joint_limits["hip_pitch_limit"])
    {
      RCLCPP_WARN(rclcpp::get_logger("ik_node"), "Hip pitch joint angle out of range");
      return false;
    }
    if (abs(leg_joints.front_right_leg.elbow_joint) > _joint_limits["elbow_limit"] ||
        abs(leg_joints.front_left_leg.elbow_joint)  > _joint_limits["elbow_limit"] ||
        abs(leg_joints.back_left_leg.elbow_joint)   > _joint_limits["elbow_limit"] ||
        abs(leg_joints.back_right_leg.elbow_joint)  > _joint_limits["elbow_limit"])
    {
      RCLCPP_WARN(rclcpp::get_logger("ik_node"), "Elbow joint angle out of range");
      return false;
    }
  }
  return true;
}

void InverseKinematics::publishAllJoints()
{
  auto front_right_msg = trajectory_msgs::msg::JointTrajectory();
  auto front_left_msg  = trajectory_msgs::msg::JointTrajectory();
  auto back_left_msg   = trajectory_msgs::msg::JointTrajectory();
  auto back_right_msg  = trajectory_msgs::msg::JointTrajectory();

  front_right_msg.joint_names.resize(3);
  front_right_msg.joint_names[0] = "front_right_hip_roll_joint";
  front_right_msg.joint_names[1] = "front_right_hip_pitch_joint";
  front_right_msg.joint_names[2] = "front_right_elbow_joint";
  front_right_msg.points.resize(_all_leg_joints.size());

  front_left_msg.joint_names.resize(3);
  front_left_msg.joint_names[0] = "front_left_hip_roll_joint";
  front_left_msg.joint_names[1] = "front_left_hip_pitch_joint";
  front_left_msg.joint_names[2] = "front_left_elbow_joint";
  front_left_msg.points.resize(_all_leg_joints.size());
  
  back_left_msg.joint_names.resize(3);
  back_left_msg.joint_names[0] = "back_left_hip_roll_joint";
  back_left_msg.joint_names[1] = "back_left_hip_pitch_joint";
  back_left_msg.joint_names[2] = "back_left_elbow_joint";
  back_left_msg.points.resize(_all_leg_joints.size());
  
  back_right_msg.joint_names.resize(3);
  back_right_msg.joint_names[0] = "back_right_hip_roll_joint";
  back_right_msg.joint_names[1] = "back_right_hip_pitch_joint";
  back_right_msg.joint_names[2] = "back_right_elbow_joint";
  back_right_msg.points.resize(_all_leg_joints.size());

  for (int i=0; i<(int)_all_leg_joints.size(); i++)
  {
    front_right_msg.points[i].positions.resize(3);
    front_right_msg.points[i].positions[0] = _all_leg_joints[i].front_right_leg.hip_roll_joint;
    front_right_msg.points[i].positions[1] = _all_leg_joints[i].front_right_leg.hip_pitch_joint;
    front_right_msg.points[i].positions[2] = _all_leg_joints[i].front_right_leg.elbow_joint;

    front_left_msg.points[i].positions.resize(3);
    front_left_msg.points[i].positions[0] = _all_leg_joints[i].front_left_leg.hip_roll_joint;
    front_left_msg.points[i].positions[1] = _all_leg_joints[i].front_left_leg.hip_pitch_joint;
    front_left_msg.points[i].positions[2] = _all_leg_joints[i].front_left_leg.elbow_joint;
    
    back_left_msg.points[i].positions.resize(3);
    back_left_msg.points[i].positions[0] = _all_leg_joints[i].back_left_leg.hip_roll_joint;
    back_left_msg.points[i].positions[1] = _all_leg_joints[i].back_left_leg.hip_pitch_joint;
    back_left_msg.points[i].positions[2] = _all_leg_joints[i].back_left_leg.elbow_joint;

    back_right_msg.points[i].positions.resize(3);
    back_right_msg.points[i].positions[0] = _all_leg_joints[i].back_right_leg.hip_roll_joint;
    back_right_msg.points[i].positions[1] = _all_leg_joints[i].back_right_leg.hip_pitch_joint;
    back_right_msg.points[i].positions[2] = _all_leg_joints[i].back_right_leg.elbow_joint;
  }

  _front_right_trajectory_publisher->publish(front_right_msg);
  _front_left_trajectory_publisher->publish(front_left_msg);
  _back_left_trajectory_publisher->publish(back_left_msg);
  _back_right_trajectory_publisher->publish(back_right_msg);

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<InverseKinematics>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
