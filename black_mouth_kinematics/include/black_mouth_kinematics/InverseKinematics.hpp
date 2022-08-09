#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik.hpp"
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
  void IKCallback(const black_mouth_kinematics::msg::BodyLegIK::SharedPtr msg);
  void publishAllJoints();
  
  void computeIK();
  bool checkJointAngles();
  void computeIKAndPublishJoints();

  rclcpp::CallbackGroup::SharedPtr _callback_group;
  rclcpp::SubscriptionOptions _sub_options;

  rclcpp::Subscription<black_mouth_kinematics::msg::BodyLegIK>::SharedPtr _cmd_subscriber;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _front_right_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _front_left_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _back_left_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _back_right_trajectory_publisher;

  rclcpp::Client<black_mouth_kinematics::srv::InvKinematics>::SharedPtr _ik_client;

  black_mouth_kinematics::msg::BodyLegIK _cmd_ik_msg;
  black_mouth_kinematics::msg::AllLegJoints _all_leg_joints;

  // TO-DO: Put these values in a config file
  float _hip_roll_limit  = M_PI/6;
  float _hip_pitch_limit = M_PI/4;
  float _elbow_limit     = M_PI/5;

};
