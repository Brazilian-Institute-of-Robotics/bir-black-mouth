#include "rclcpp/rclcpp.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "caramel_kinematics/msg/all_leg_joints.hpp"
#include "caramel_kinematics/srv/inv_kinematics.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/empty.hpp"

#include <memory>

class InverseKinematics : public rclcpp::Node
{
public:
  InverseKinematics();
  ~InverseKinematics();

private:
  void IKCallback(const caramel_kinematics::msg::BodyLegIKTrajectory::SharedPtr msg);
  void defaultPoseCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void publishAllJoints();
  
  void computeIK();
  bool checkJointAngles();
  void computeIKAndPublishJoints();

  rclcpp::CallbackGroup::SharedPtr _callback_group;
  rclcpp::SubscriptionOptions _sub_options;

  rclcpp::Subscription<caramel_kinematics::msg::BodyLegIKTrajectory>::SharedPtr _cmd_subscriber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _default_pose_subscriber;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _front_right_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _front_left_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _back_left_trajectory_publisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _back_right_trajectory_publisher;

  rclcpp::Client<caramel_kinematics::srv::InvKinematics>::SharedPtr _ik_client;

  caramel_kinematics::msg::BodyLegIKTrajectory _cmd_ik_msg;
  std::vector<caramel_kinematics::msg::AllLegJoints> _all_leg_joints;

  std::map<std::string, double> _default_joint_limits;
  std::map<std::string, double> _joint_limits;

};
