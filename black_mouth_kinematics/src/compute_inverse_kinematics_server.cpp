#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"

#include <memory>

void computeInvKinematics(const std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Request> request,
                          std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Response> responde)
{
  responde->front_right_leg.hip_roll_joint  = request->body_rotation.x;
  responde->front_right_leg.hip_pitch_joint = request->body_rotation.y;
  responde->front_right_leg.elbow_joint     = request->body_rotation.z;

  responde->front_left_leg.hip_roll_joint  = 2*request->body_rotation.x;
  responde->front_left_leg.hip_pitch_joint = 2*request->body_rotation.y;
  responde->front_left_leg.elbow_joint     = 2*request->body_rotation.z;

  responde->back_left_leg.hip_roll_joint  = 3*request->body_rotation.x;
  responde->back_left_leg.hip_pitch_joint = 3*request->body_rotation.y;
  responde->back_left_leg.elbow_joint     = 3*request->body_rotation.z;

  responde->back_right_leg.hip_roll_joint  = 4*request->body_rotation.x;
  responde->back_right_leg.hip_pitch_joint = 4*request->body_rotation.y;
  responde->back_right_leg.elbow_joint     = 4*request->body_rotation.z;

  RCLCPP_INFO(rclcpp::get_logger("ik_server"), "Incoming request for Inverse Kinematics");

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("compute_inverse_kinematics_server");
  
  rclcpp::Service<black_mouth_kinematics::srv::InvKinematics>::SharedPtr service = 
    node->create_service<black_mouth_kinematics::srv::InvKinematics>("compute_inverse_kinematics", &computeInvKinematics);

  RCLCPP_INFO(rclcpp::get_logger("ik_server"), "Ready to compute Inverse Kinematics");

  rclcpp::spin(node);
  rclcpp::shutdown();

}
