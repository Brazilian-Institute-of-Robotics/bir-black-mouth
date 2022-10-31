#include "rclcpp/rclcpp.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("inv_kinematics_trajectory_node");

  rclcpp::Publisher<caramel_kinematics::msg::BodyLegIKTrajectory>::SharedPtr publisher = 
    node->create_publisher<caramel_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);

  auto msg = caramel_kinematics::msg::BodyLegIKTrajectory();
  msg.body_leg_ik_trajectory.resize(10);
  msg.time_from_start.resize(10);

  msg.time_from_start[0].sec = 1.0;
  msg.time_from_start[0].nanosec = 0;
  msg.body_leg_ik_trajectory[0].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[0].body_position.x = 0.0;
  msg.body_leg_ik_trajectory[0].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[0].leg_points.front_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[0].leg_points.front_right_leg.z = 0.0;
  msg.body_leg_ik_trajectory[0].leg_points.front_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[0].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[0].leg_points.back_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[0].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[0].leg_points.back_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[0].leg_points.back_right_leg.z = 0.0;



  msg.time_from_start[1].sec = 1.0;
  msg.time_from_start[1].nanosec = 500000000;
  msg.body_leg_ik_trajectory[1].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[1].body_position.x = 0.0;
  msg.body_leg_ik_trajectory[1].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[1].leg_points.front_right_leg.x = 0.03;
  msg.body_leg_ik_trajectory[1].leg_points.front_right_leg.z = 0.05;
  msg.body_leg_ik_trajectory[1].leg_points.front_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[1].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[1].leg_points.back_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[1].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[1].leg_points.back_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[1].leg_points.back_right_leg.z = 0.0;

  msg.time_from_start[2].sec = 2.0;
  msg.time_from_start[2].nanosec = 0;
  msg.body_leg_ik_trajectory[2].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[2].body_position.x = 0.03;
  msg.body_leg_ik_trajectory[2].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[2].leg_points.front_right_leg.x = 0.06;
  msg.body_leg_ik_trajectory[2].leg_points.front_right_leg.z = 0.0;
  msg.body_leg_ik_trajectory[2].leg_points.front_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[2].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[2].leg_points.back_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[2].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[2].leg_points.back_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[2].leg_points.back_right_leg.z = 0.0;

  msg.time_from_start[3].sec = 2.0;
  msg.time_from_start[3].nanosec = 500000000;
  msg.body_leg_ik_trajectory[3].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[3].body_position.x = 0.03;
  msg.body_leg_ik_trajectory[3].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[3].leg_points.front_right_leg.x = 0.06;
  msg.body_leg_ik_trajectory[3].leg_points.front_right_leg.z = 0.00;
  msg.body_leg_ik_trajectory[3].leg_points.front_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[3].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[3].leg_points.back_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[3].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[3].leg_points.back_right_leg.x = 0.03;
  msg.body_leg_ik_trajectory[3].leg_points.back_right_leg.z = 0.05;

  msg.time_from_start[4].sec = 3.0;
  msg.time_from_start[4].nanosec = 0;
  msg.body_leg_ik_trajectory[4].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[4].body_position.x = 0.03;
  msg.body_leg_ik_trajectory[4].body_position.y = 0.025;
  msg.body_leg_ik_trajectory[4].leg_points.front_right_leg.x = 0.06;
  msg.body_leg_ik_trajectory[4].leg_points.front_right_leg.z = 0.0;
  msg.body_leg_ik_trajectory[4].leg_points.front_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[4].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[4].leg_points.back_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[4].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[4].leg_points.back_right_leg.x = 0.06;
  msg.body_leg_ik_trajectory[4].leg_points.back_right_leg.z = 0.0;



  msg.time_from_start[5].sec = 5.0;
  msg.time_from_start[5].nanosec = 500000000;
  msg.body_leg_ik_trajectory[5].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[5].body_position.x = -0.03;
  msg.body_leg_ik_trajectory[5].body_position.y = -0.025;
  msg.body_leg_ik_trajectory[5].leg_points.front_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[5].leg_points.front_right_leg.z = 0.0;
  msg.body_leg_ik_trajectory[5].leg_points.front_left_leg.x = -0.06;
  msg.body_leg_ik_trajectory[5].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[5].leg_points.back_left_leg.x = -0.06;
  msg.body_leg_ik_trajectory[5].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[5].leg_points.back_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[5].leg_points.back_right_leg.z = 0.0;



  msg.time_from_start[6].sec = 6.0;
  msg.time_from_start[6].nanosec = 0;
  msg.body_leg_ik_trajectory[6].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[6].body_position.x = -0.03;
  msg.body_leg_ik_trajectory[6].body_position.y = -0.025;
  msg.body_leg_ik_trajectory[6].leg_points.front_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[6].leg_points.front_right_leg.z = 0.0;
  msg.body_leg_ik_trajectory[6].leg_points.front_left_leg.x = -0.03;
  msg.body_leg_ik_trajectory[6].leg_points.front_left_leg.z = 0.05;
  msg.body_leg_ik_trajectory[6].leg_points.back_left_leg.x = -0.06;
  msg.body_leg_ik_trajectory[6].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[6].leg_points.back_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[6].leg_points.back_right_leg.z = 0.0;

  msg.time_from_start[7].sec = 6.0;
  msg.time_from_start[7].nanosec = 500000000;
  msg.body_leg_ik_trajectory[7].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[7].body_position.x = 0.0;
  msg.body_leg_ik_trajectory[7].body_position.y = -0.025;
  msg.body_leg_ik_trajectory[7].leg_points.front_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[7].leg_points.front_right_leg.z = 0.0;
  msg.body_leg_ik_trajectory[7].leg_points.front_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[7].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[7].leg_points.back_left_leg.x = -0.03;
  msg.body_leg_ik_trajectory[7].leg_points.back_left_leg.z = 0.05;
  msg.body_leg_ik_trajectory[7].leg_points.back_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[7].leg_points.back_right_leg.z = 0.0;

  msg.time_from_start[8].sec = 7.0;
  msg.time_from_start[8].nanosec = 0;
  msg.body_leg_ik_trajectory[8].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;
  msg.body_leg_ik_trajectory[8].body_position.x = 0.0;
  msg.body_leg_ik_trajectory[8].body_position.y = -0.025;
  msg.body_leg_ik_trajectory[8].leg_points.front_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[8].leg_points.front_right_leg.z = 0.0;
  msg.body_leg_ik_trajectory[8].leg_points.front_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[8].leg_points.front_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[8].leg_points.back_left_leg.x = 0.0;
  msg.body_leg_ik_trajectory[8].leg_points.back_left_leg.z = 0.0;
  msg.body_leg_ik_trajectory[8].leg_points.back_right_leg.x = 0.0;
  msg.body_leg_ik_trajectory[8].leg_points.back_right_leg.z = 0.0;

  msg.time_from_start[9].sec = 8.0;
  msg.time_from_start[9].nanosec = 0;
  msg.body_leg_ik_trajectory[9].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;

  publisher->publish(msg);

  rclcpp::shutdown();

  return 0;
}
