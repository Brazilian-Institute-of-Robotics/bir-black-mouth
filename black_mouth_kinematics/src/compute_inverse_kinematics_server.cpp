#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <memory>
#include <cmath>

float L1 = 0.0483;
float L2 = 0.120;
float L3 = 0.120;

black_mouth_kinematics::msg::LegJoints getLegIK(geometry_msgs::msg::Point point)
{

  float a = sqrt(pow(point.y, 2) + pow(point.z, 2) - pow(L1, 2));
  float A = (pow(a, 2) + pow(point.x, 2) + pow(L2, 2) - pow(L3, 2)) / (2*L2*sqrt(pow(a,2) + pow(point.x, 2)));
  float B = (pow(a, 2) + pow(point.x, 2) - pow(L2, 2) - pow(L3, 2)) / (2*L2*L3);

  black_mouth_kinematics::msg::LegJoints leg_joints;
  leg_joints.hip_roll_joint  = atan2(point.y, point.z) - atan2(L1, a);
  leg_joints.hip_pitch_joint = M_PI_2 - atan2(a, point.x) - atan2(sqrt(1 - pow(A, 2)), A);
  leg_joints.elbow_joint     = atan2(sqrt(1 - pow(B, 2)), B);

  return leg_joints;

}

void computeInvKinematics(const std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Request> request,
                          std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Response> responde)
{

  RCLCPP_INFO(rclcpp::get_logger("ik_server"), "Incoming request for Inverse Kinematics");

  responde->front_right_leg = getLegIK(request->front_right_leg);
  responde->front_left_leg  = getLegIK(request->front_left_leg);
  responde->back_left_leg   = getLegIK(request->back_left_leg);
  responde->back_right_leg  = getLegIK(request->back_right_leg);

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
