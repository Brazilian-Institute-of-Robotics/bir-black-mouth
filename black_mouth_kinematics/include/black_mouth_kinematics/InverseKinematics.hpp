#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "black_mouth_kinematics/msg/all_leg_joints.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <memory>

class InverseKinematics : public rclcpp::Node
{
public:
  InverseKinematics();
  ~InverseKinematics();

private:
  void IKCallback(const black_mouth_kinematics::msg::BodyLegIKTrajectory::SharedPtr msg);
  void publishAllJoints();
  
  void computeIK();
  bool checkJointAngles();
  void computeIKAndPublishJoints();

  rclcpp::CallbackGroup::SharedPtr _callback_group;
  rclcpp::SubscriptionOptions _sub_options;

  rclcpp::Subscription<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr _cmd_subscriber;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _front_right_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _front_left_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _back_left_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _back_right_trajectory_publisher;

  rclcpp::Client<black_mouth_kinematics::srv::InvKinematics>::SharedPtr _ik_client;

  black_mouth_kinematics::msg::BodyLegIKTrajectory _cmd_ik_msg;
  std::vector<black_mouth_kinematics::msg::AllLegJoints> _all_leg_joints;

  std::map<std::string, double> _default_joint_limits;
  std::map<std::string, double> _joint_limits;

};
