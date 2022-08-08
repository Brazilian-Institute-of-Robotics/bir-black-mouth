#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/InverseKinematics.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;

InverseKinematics::InverseKinematics() : Node("inverse_kinematics_node")
{
  RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Node initialized");

  _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _sub_options.callback_group = _callback_group;

  _cmd_subscriber = this->create_subscription<black_mouth_kinematics::msg::BodyLegIK>("cmd_ik", 10, 
                          std::bind(&InverseKinematics::IKCallback, this, _1), _sub_options);

  // This publisher will not exist. It will be the 4 publishers for each controller instead
  _publisher = this->create_publisher<black_mouth_kinematics::msg::AllLegJoints>("/all_leg_joints", 10);

  _ik_client = this->create_client<black_mouth_kinematics::srv::InvKinematics>("compute_inverse_kinematics", 
                                                                                rmw_qos_profile_services_default, 
                                                                                _callback_group);
  
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

void InverseKinematics::IKCallback(const black_mouth_kinematics::msg::BodyLegIK::SharedPtr msg)
{
  this->_cmd_ik_msg = *msg;

  this->computeIKAndPublishJoints();
}

void InverseKinematics::computeIKAndPublishJoints()
{
  this->computeIK();
  this->checkJointAngles();
  this->publishAllJoints();
}

void InverseKinematics::computeIK()
{
  // auto start = std::chrono::steady_clock::now();

  auto request = std::make_shared<black_mouth_kinematics::srv::InvKinematics::Request>();
  request->body_leg_ik = this->_cmd_ik_msg;
  
  auto result = this->_ik_client->async_send_request(request);
  std::future_status status = result.wait_for(3ms);
  if (status == std::future_status::ready)
    _all_leg_joints = result.get()->leg_joints;
  else
    RCLCPP_INFO(rclcpp::get_logger("ik_client"), "Failed to comput leg joints, timeout");

  // auto end = std::chrono::steady_clock::now();
  // std::cout << "Elapsed time in microseconds: "
            // << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
            // << " µs" << std::endl;

}

bool InverseKinematics::checkJointAngles()
{

  if (abs(_all_leg_joints.front_right_leg.hip_roll_joint) > _hip_roll_limit ||
      abs(_all_leg_joints.front_left_leg.hip_roll_joint)  > _hip_roll_limit ||
      abs(_all_leg_joints.back_left_leg.hip_roll_joint)   > _hip_roll_limit ||
      abs(_all_leg_joints.back_right_leg.hip_roll_joint)  > _hip_roll_limit)
  {
    RCLCPP_WARN(rclcpp::get_logger("ik_node"), "Hip roll joint angle out of range");
    return false;
  }
  if (abs(_all_leg_joints.front_right_leg.hip_pitch_joint) > _hip_pitch_limit ||
      abs(_all_leg_joints.front_left_leg.hip_pitch_joint)  > _hip_pitch_limit ||
      abs(_all_leg_joints.back_left_leg.hip_pitch_joint)   > _hip_pitch_limit ||
      abs(_all_leg_joints.back_right_leg.hip_pitch_joint)  > _hip_pitch_limit)
  {
    RCLCPP_WARN(rclcpp::get_logger("ik_node"), "Hip pitch joint angle out of range");
    return false;
  }
  if (abs(_all_leg_joints.front_right_leg.elbow_joint) > _elbow_limit ||
      abs(_all_leg_joints.front_left_leg.elbow_joint)  > _elbow_limit ||
      abs(_all_leg_joints.back_left_leg.elbow_joint)   > _elbow_limit ||
      abs(_all_leg_joints.back_right_leg.elbow_joint)  > _elbow_limit)
  {
    RCLCPP_WARN(rclcpp::get_logger("ik_node"), "Elbow joint angle out of range");
    return false;
  }

  return true;
}

void InverseKinematics::publishAllJoints()
{

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
