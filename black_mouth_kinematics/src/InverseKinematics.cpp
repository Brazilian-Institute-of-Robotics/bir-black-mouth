#include <memory>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/InverseKinematics.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

InverseKinematics::InverseKinematics() : Node("inverse_kinematics_node")
{
  RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Node initialized");

  _cmd_subscriber = this->create_subscription<black_mouth_kinematics::msg::BodyLegIK>("cmd_ik", 10, 
                      std::bind(&InverseKinematics::IKCallback, this, _1));
  // This publisher will not exist. It will be the 4 publishers for each controller instead
  _publisher = this->create_publisher<black_mouth_kinematics::msg::AllLegJoints>("/all_leg_joints", 10);

  _ik_client = this->create_client<black_mouth_kinematics::srv::InvKinematics>("compute_inverse_kinematics");

  while(!_ik_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}

InverseKinematics::~InverseKinematics()
{
}

void InverseKinematics::IKCallback(const black_mouth_kinematics::msg::BodyLegIK::SharedPtr msg)
{
  this->_cmd_ik_msg = *msg;

  RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Subscriber");

  // TODO: Remove from CB
  this->computeIK();
  this->checkJointAngles();
  this->publishAllJoints();
}

void InverseKinematics::publishAllJoints()
{

}

void InverseKinematics::computeIK()
{
  auto request = std::make_shared<black_mouth_kinematics::srv::InvKinematics::Request>();
  request->body_leg_ik = this->_cmd_ik_msg;
  
  RCLCPP_INFO(rclcpp::get_logger("ik_client"), "Computing leg joints");

  // FIX: client dont work 
  auto result = this->_ik_client->async_send_request(request);
  result.wait();

  _all_leg_joints = result.get()->leg_joints;

  RCLCPP_INFO(rclcpp::get_logger("ik_client"), "Leg joints computed");

}

void InverseKinematics::checkJointAngles()
{

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematics>());
  rclcpp::shutdown();
  return 0;
}
