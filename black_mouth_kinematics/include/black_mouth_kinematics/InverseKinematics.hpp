#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/msg/all_leg_joints.hpp"
#include "black_mouth_kinematics/msg/all_leg_points.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"

class InverseKinematics : public rclcpp::Node
{
public:
  InverseKinematics();
  ~InverseKinematics();

private:
  void IKCallback(const black_mouth_kinematics::msg::BodyLegIK::SharedPtr msg);
  void publishAllJoints();
  
  void computeIK();
  void checkJointAngles();

  rclcpp::Subscription<black_mouth_kinematics::msg::BodyLegIK>::SharedPtr _cmd_subscriber;
  rclcpp::Publisher<black_mouth_kinematics::msg::AllLegJoints>::SharedPtr _publisher;
  // This publisher will not exist. It will be the 4 publishers for each leg controller instead

  rclcpp::Client<black_mouth_kinematics::srv::InvKinematics>::SharedPtr _ik_client;

  black_mouth_kinematics::msg::BodyLegIK _cmd_ik_msg;
  black_mouth_kinematics::msg::AllLegJoints _all_leg_joints;
  
};
