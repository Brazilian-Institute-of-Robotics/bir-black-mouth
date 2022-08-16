#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "geometry_msgs/msg/point.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik.hpp"

#include <memory>
#include <cmath>

std::map<std::string, double> body_dimensions;
std::map<std::string, double> leg_dimensions;

struct legsEigenTransformations
{
  Eigen::Matrix4d Tm_front_right;
  Eigen::Matrix4d Tm_front_left;
  Eigen::Matrix4d Tm_back_left;
  Eigen::Matrix4d Tm_back_right;
};

void getQuadrupedParameters(rclcpp::Node::SharedPtr node)
{
  std::map<std::string, double> default_body_dimensions{ {"L", 0.2}, {"W", 0.15}, {"H", 0.15} };
  std::map<std::string, double> default_leg_dimensions{ {"L1", 0.05}, {"L2", 0.10}, {"L3", 0.10} };
  node->declare_parameters("body_dimensions", default_body_dimensions);
  node->declare_parameters("leg_dimensions", default_leg_dimensions);
  node->get_parameters("body_dimensions", body_dimensions);
  node->get_parameters("leg_dimensions", leg_dimensions);
}

Eigen::Matrix4d getTranslationMatrix(const float x, const float y, const float z)
{
  Eigen::Matrix4d translation_matrix;
  translation_matrix << 1.0, 0.0, 0.0, x,
                        0.0, 1.0, 0.0, y,
                        0.0, 0.0, 1.0, z,
                        0.0, 0.0, 0.0, 1.0;
  return translation_matrix;
}

Eigen::Matrix4d getTransformationMatrix(const geometry_msgs::msg::Vector3 translation_vector3, 
                                        const geometry_msgs::msg::Vector3 rotation_vector3)
{
  Eigen::Vector3d translation;
  tf2::fromMsg(translation_vector3, translation);

  Eigen::AngleAxisd rollRotation(rotation_vector3.x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchRotation(rotation_vector3.y, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawRotation(rotation_vector3.z, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = rollRotation*pitchRotation*yawRotation;
  Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

  Eigen::Matrix4d Tm = Eigen::MatrixXd::Identity(4, 4);
  Tm.block<3, 3>(0, 0) = rotationMatrix;
  Tm.block<3, 1>(0, 3) = translation;

  return Tm;
}

legsEigenTransformations getBodyIK(const geometry_msgs::msg::Vector3 body_position, 
                                   const geometry_msgs::msg::Vector3 body_rotation)
{
  Eigen::Matrix4d Tm = getTransformationMatrix(body_position, body_rotation);

  Eigen::Matrix4d T_front_right = getTranslationMatrix( body_dimensions["L"]/2, -body_dimensions["W"]/2, 0.0);
  Eigen::Matrix4d T_front_left  = getTranslationMatrix( body_dimensions["L"]/2,  body_dimensions["W"]/2, 0.0);
  Eigen::Matrix4d T_back_left   = getTranslationMatrix(-body_dimensions["L"]/2,  body_dimensions["W"]/2, 0.0);
  Eigen::Matrix4d T_back_right  = getTranslationMatrix(-body_dimensions["L"]/2, -body_dimensions["W"]/2, 0.0);

  legsEigenTransformations transformations;
  transformations.Tm_front_right = Tm*T_front_right;
  transformations.Tm_front_left  = Tm*T_front_left;
  transformations.Tm_back_left   = Tm*T_back_left;
  transformations.Tm_back_right  = Tm*T_back_right;

  return transformations;
}

black_mouth_kinematics::msg::LegJoints getLegIK(const geometry_msgs::msg::Point point, bool left=false)
{
  int reflect = left ? 1 : -1;

  float a = sqrt(pow(point.y, 2) + pow(point.z, 2) - pow(leg_dimensions["L1"], 2));
  float A = (pow(a, 2) + pow(point.x, 2) + pow(leg_dimensions["L2"], 2) - pow(leg_dimensions["L3"], 2)) / (2*leg_dimensions["L2"]*sqrt(pow(a,2) + pow(point.x, 2)));
  float B = (pow(a, 2) + pow(point.x, 2) - pow(leg_dimensions["L2"], 2) - pow(leg_dimensions["L3"], 2)) / (2*leg_dimensions["L2"]*leg_dimensions["L3"]);

  float theta1 = atan2(point.y, -point.z) - atan2(reflect*leg_dimensions["L1"], a);
  float theta2 = M_PI_2 - atan2(a, point.x) - atan2(sqrt(1 - pow(A, 2)), A);
  float theta3 = atan2(sqrt(1 - pow(B, 2)), B);

  black_mouth_kinematics::msg::LegJoints leg_joints;
  leg_joints.hip_roll_joint  = theta1;
  leg_joints.hip_pitch_joint = -(theta2 + M_PI_4);
  leg_joints.elbow_joint     = -(theta3 - M_PI_2);
  
  return leg_joints;
}

void computeInvKinematics(const std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Request> request,
                                std::shared_ptr<black_mouth_kinematics::srv::InvKinematics::Response> responde)
{

  // RCLCPP_INFO(rclcpp::get_logger("ik_server"), "Incoming request for Inverse Kinematics");

  Eigen::Vector3d front_right_foot;
  Eigen::Vector3d front_left_foot;
  Eigen::Vector3d back_left_foot;
  Eigen::Vector3d back_right_foot;

  tf2::fromMsg(request->body_leg_ik.leg_points.front_right_leg, front_right_foot);
  tf2::fromMsg(request->body_leg_ik.leg_points.front_left_leg, front_left_foot);
  tf2::fromMsg(request->body_leg_ik.leg_points.back_left_leg, back_left_foot);
  tf2::fromMsg(request->body_leg_ik.leg_points.back_right_leg, back_right_foot);

  if (request->body_leg_ik.leg_points.reference_link == black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE)
  {
    front_right_foot = (getTranslationMatrix( body_dimensions["L"]/2, -(body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*front_right_foot.homogeneous()).head<3>();
    front_left_foot  = (getTranslationMatrix( body_dimensions["L"]/2,  (body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*front_left_foot.homogeneous()).head<3>();
    back_left_foot   = (getTranslationMatrix(-body_dimensions["L"]/2,  (body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*back_left_foot.homogeneous()).head<3>();
    back_right_foot  = (getTranslationMatrix(-body_dimensions["L"]/2, -(body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*back_right_foot.homogeneous()).head<3>();
  }
  else if (request->body_leg_ik.leg_points.reference_link == black_mouth_kinematics::msg::AllLegPoints::HIP_LINK_AS_REFERENCE)
  {
    front_right_foot = (getTranslationMatrix( body_dimensions["L"]/2, -body_dimensions["W"]/2, 0.0)*front_right_foot.homogeneous()).head<3>();
    front_left_foot  = (getTranslationMatrix( body_dimensions["L"]/2,  body_dimensions["W"]/2, 0.0)*front_left_foot.homogeneous()).head<3>();
    back_left_foot   = (getTranslationMatrix(-body_dimensions["L"]/2,  body_dimensions["W"]/2, 0.0)*back_left_foot.homogeneous()).head<3>();
    back_right_foot  = (getTranslationMatrix(-body_dimensions["L"]/2, -body_dimensions["W"]/2, 0.0)*back_right_foot.homogeneous()).head<3>();
  }

  legsEigenTransformations bodyIK = getBodyIK(request->body_leg_ik.body_position, request->body_leg_ik.body_rotation);

  Eigen::Vector4d front_right_IK = bodyIK.Tm_front_right.inverse()*front_right_foot.homogeneous();
  Eigen::Vector4d front_left_IK  = bodyIK.Tm_front_left.inverse()*front_left_foot.homogeneous();
  Eigen::Vector4d back_left_IK   = bodyIK.Tm_back_left.inverse()*back_left_foot.homogeneous();
  Eigen::Vector4d back_right_IK  = bodyIK.Tm_back_right.inverse()*back_right_foot.homogeneous();

  geometry_msgs::msg::Point front_right_point = tf2::toMsg(Eigen::Vector3d(front_right_IK.head(3)));
  geometry_msgs::msg::Point front_left_point  = tf2::toMsg(Eigen::Vector3d(front_left_IK.head(3)));
  geometry_msgs::msg::Point back_left_point   = tf2::toMsg(Eigen::Vector3d(back_left_IK.head(3)));
  geometry_msgs::msg::Point back_right_point  = tf2::toMsg(Eigen::Vector3d(back_right_IK.head(3)));

  responde->leg_joints.front_right_leg = getLegIK(front_right_point, false);
  responde->leg_joints.front_left_leg  = getLegIK(front_left_point, true);
  responde->leg_joints.back_left_leg   = getLegIK(back_left_point, true);
  responde->leg_joints.back_right_leg  = getLegIK(back_right_point, false);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("compute_inverse_kinematics_server");
  
  getQuadrupedParameters(node);
  
  rclcpp::Service<black_mouth_kinematics::srv::InvKinematics>::SharedPtr service = 
    node->create_service<black_mouth_kinematics::srv::InvKinematics>("compute_inverse_kinematics", &computeInvKinematics);

  RCLCPP_INFO(rclcpp::get_logger("ik_server"), "Ready to compute Inverse Kinematics");

  rclcpp::spin(node);
  rclcpp::shutdown();

}
