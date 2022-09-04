#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik_trajectory.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("inv_kinematics_trajectory_node");

  rclcpp::Publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>::SharedPtr publisher = 
    node->create_publisher<black_mouth_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);

  auto msg = black_mouth_kinematics::msg::BodyLegIKTrajectory();
  msg.body_leg_ik_trajectory.resize(6);
  msg.time_from_start.resize(6);


  msg.time_from_start[0].sec = 1.0;
  msg.body_leg_ik_trajectory[0].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[0].body_position.y = 0.025;
  msg.time_from_start[0].nanosec = 0;

  msg.time_from_start[1].sec = 1.0;
  msg.time_from_start[1].nanosec = 400000000;
  msg.body_leg_ik_trajectory[1].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[1].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[1].leg_points.front_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[1].leg_points.front_right_leg.z = 0.025;

  msg.time_from_start[2].sec = 1.0;
  msg.time_from_start[2].nanosec = 800000000;
  msg.body_leg_ik_trajectory[2].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[2].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[2].leg_points.front_right_leg.x = 0.025;
  msg.body_leg_ik_trajectory[2].leg_points.front_right_leg.z = 0.05;

  msg.time_from_start[3].sec = 1.0;
  msg.time_from_start[3].nanosec = 1200000000;
  msg.body_leg_ik_trajectory[3].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[3].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[3].leg_points.front_right_leg.x = 0.05;
  msg.body_leg_ik_trajectory[3].leg_points.front_right_leg.z = 0.025;

  msg.time_from_start[4].sec = 1.0;
  msg.time_from_start[4].nanosec = 1600000000;
  msg.body_leg_ik_trajectory[4].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[4].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[4].leg_points.front_right_leg.x = 0.05;
  msg.body_leg_ik_trajectory[4].leg_points.front_right_leg.z = -0.0025;

  msg.time_from_start[5].sec = 1.0;
  msg.time_from_start[5].nanosec = 2000000000;
  msg.body_leg_ik_trajectory[5].leg_points.reference_link = black_mouth_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[5].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[5].body_position.x = 0.025;
  msg.body_leg_ik_trajectory[5].leg_points.front_right_leg.x = 0.00;
  msg.body_leg_ik_trajectory[5].leg_points.front_right_leg.z = -0.005;

  publisher->publish(msg);

  rclcpp::shutdown();

  return 0;
}
