#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <memory>
#include <cmath>

float L = 0.229;
float W = 0.14;
float L1 = 0.0483;
float L2 = 0.120;
float L3 = 0.120;

black_mouth_kinematics::msg::LegJoints getLegIK(const geometry_msgs::msg::Point point, bool left=false)
{

  float a = sqrt(pow(point.y, 2) + pow(point.z, 2) - pow(L1, 2));
  float A = (pow(a, 2) + pow(point.x, 2) + pow(L2, 2) - pow(L3, 2)) / (2*L2*sqrt(pow(a,2) + pow(point.x, 2)));
  float B = (pow(a, 2) + pow(point.x, 2) - pow(L2, 2) - pow(L3, 2)) / (2*L2*L3);

  float theta1 = atan2(point.y, -point.z) - atan2(L1, a);
  float theta2 = M_PI_2 - atan2(a, point.x) - atan2(sqrt(1 - pow(A, 2)), A);
  float theta3 = atan2(sqrt(1 - pow(B, 2)), B);

  black_mouth_kinematics::msg::LegJoints leg_joints;
  if (left) leg_joints.hip_roll_joint  = theta1;
  else leg_joints.hip_roll_joint  = -theta1;
  leg_joints.hip_pitch_joint = -(theta2 + M_PI_4);
  leg_joints.elbow_joint     = -(theta3 - M_PI_2);
  
  return leg_joints;
}

void computeInvKinematics(const std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Request> request,
                                std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Response> responde)
{

  RCLCPP_INFO(rclcpp::get_logger("ik_server"), "Incoming request for Inverse Kinematics");

  geometry_msgs::msg::Point front_right_foot;
  geometry_msgs::msg::Point front_left_foot;
  geometry_msgs::msg::Point back_left_foot;
  geometry_msgs::msg::Point back_right_foot;
  
  // Compute Body IK
  front_right_foot.x = request->front_right_leg.x - request->body_position.x;
  front_right_foot.y = request->front_right_leg.y + request->body_position.y;
  front_right_foot.z = request->front_right_leg.z - request->body_position.z;

  front_left_foot.x = request->front_left_leg.x - request->body_position.x;
  front_left_foot.y = request->front_left_leg.y - request->body_position.y;
  front_left_foot.z = request->front_left_leg.z - request->body_position.z;

  back_left_foot.x = request->back_left_leg.x - request->body_position.x;
  back_left_foot.y = request->back_left_leg.y - request->body_position.y;
  back_left_foot.z = request->back_left_leg.z - request->body_position.z;

  back_right_foot.x = request->back_right_leg.x - request->body_position.x;
  back_right_foot.y = request->back_right_leg.y + request->body_position.y;
  back_right_foot.z = request->back_right_leg.z - request->body_position.z;

  responde->front_right_leg = getLegIK(front_right_foot, false);
  responde->front_left_leg  = getLegIK(front_left_foot, true);
  responde->back_left_leg   = getLegIK(back_left_foot, true);
  responde->back_right_leg  = getLegIK(back_right_foot, false);

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
