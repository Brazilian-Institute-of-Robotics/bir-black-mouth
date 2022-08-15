#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("inv_kinematics_trajectory_node");

  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr publisher = 
    node->create_publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);

  auto msg = black_mouth_kinematics::msg::BodyLegIKTrajectory();
  msg.body_leg_ik_trajectory.resize(8);
  msg.time_from_start.resize(8);

  msg.time_from_start[0].sec = 1.0;
  msg.body_leg_ik_trajectory[0].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[0].leg_points.front_right_leg.x = -0.01;
  msg.body_leg_ik_trajectory[0].leg_points.front_right_leg.z = 0.01;
  msg.body_leg_ik_trajectory[0].leg_points.back_left_leg.x   = -0.01;
  msg.body_leg_ik_trajectory[0].leg_points.back_left_leg.z   = 0.01;

  msg.time_from_start[1].sec = 2.0;
  msg.body_leg_ik_trajectory[1].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[1].leg_points.front_right_leg.x = 0.025;
  msg.body_leg_ik_trajectory[1].leg_points.front_right_leg.z = 0.025;
  msg.body_leg_ik_trajectory[1].leg_points.back_left_leg.x   = 0.025;
  msg.body_leg_ik_trajectory[1].leg_points.back_left_leg.z   = 0.025;

  msg.time_from_start[2].sec = 3.0;
  msg.body_leg_ik_trajectory[2].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[2].leg_points.front_right_leg.x = 0.05;
  msg.body_leg_ik_trajectory[2].leg_points.front_right_leg.z = 0.01;
  msg.body_leg_ik_trajectory[2].leg_points.back_left_leg.x   = 0.05;
  msg.body_leg_ik_trajectory[2].leg_points.back_left_leg.z   = 0.01;

  msg.time_from_start[3].sec = 4.0;
  msg.body_leg_ik_trajectory[3].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[3].leg_points.front_right_leg.x = 0.00;
  msg.body_leg_ik_trajectory[3].leg_points.front_right_leg.z = 0.00;
  msg.body_leg_ik_trajectory[3].leg_points.back_left_leg.x   = 0.00;
  msg.body_leg_ik_trajectory[3].leg_points.back_left_leg.z   = 0.00;

  //
  
  msg.time_from_start[4].sec = 5.0;
  msg.body_leg_ik_trajectory[4].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[4].leg_points.front_left_leg.x   = -0.01;
  msg.body_leg_ik_trajectory[4].leg_points.front_left_leg.z   = 0.01;
  msg.body_leg_ik_trajectory[4].leg_points.back_right_leg.x = -0.01;
  msg.body_leg_ik_trajectory[4].leg_points.back_right_leg.z = 0.01;

  msg.time_from_start[5].sec = 6.0;
  msg.body_leg_ik_trajectory[5].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[5].leg_points.front_left_leg.x   = 0.025;
  msg.body_leg_ik_trajectory[5].leg_points.front_left_leg.z   = 0.025;
  msg.body_leg_ik_trajectory[5].leg_points.back_right_leg.x = 0.025;
  msg.body_leg_ik_trajectory[5].leg_points.back_right_leg.z = 0.025;

  msg.time_from_start[6].sec = 7.0;
  msg.body_leg_ik_trajectory[6].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[6].leg_points.front_left_leg.x   = 0.05;
  msg.body_leg_ik_trajectory[6].leg_points.front_left_leg.z   = 0.01;
  msg.body_leg_ik_trajectory[6].leg_points.back_right_leg.x = 0.05;
  msg.body_leg_ik_trajectory[6].leg_points.back_right_leg.z = 0.01;

  msg.time_from_start[7].sec = 8.0;
  msg.body_leg_ik_trajectory[7].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[7].leg_points.front_left_leg.x   = 0.00;
  msg.body_leg_ik_trajectory[7].leg_points.front_left_leg.z   = 0.00;
  msg.body_leg_ik_trajectory[7].leg_points.back_right_leg.x = 0.00;
  msg.body_leg_ik_trajectory[7].leg_points.back_right_leg.z = 0.00;

  publisher->publish(msg);

  rclcpp::shutdown();

  return 0;
}
