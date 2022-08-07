#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"
#include "black_mouth_kinematics/msg/body_leg_ik.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("compute_inverse_kinematics_client");

  rclcpp::Client<black_mouth_kinematics::srv::InvKinematics>::SharedPtr client =
    node->create_client<black_mouth_kinematics::srv::InvKinematics>("compute_inverse_kinematics");

  auto request = std::make_shared<black_mouth_kinematics::srv::InvKinematics::Request>();

  request->body_leg_ik.body_position.x = 0.0;
  request->body_leg_ik.body_position.y = 0.0;
  request->body_leg_ik.body_position.z = 0.025;
  request->body_leg_ik.body_rotation.x = 0.0;
  request->body_leg_ik.body_rotation.y = 0.0;
  request->body_leg_ik.body_rotation.z = M_PI/8;

  // request->body_leg_ik.reference_link = black_mouth_kinematics::msg::BodyLegIK::BASE_LINK_AS_REFERENCE;
  // request->body_leg_ik.leg_points.front_right_leg.x = 0.115;
  // request->body_leg_ik.leg_points.front_right_leg.y = -0.118;
  // request->body_leg_ik.leg_points.front_right_leg.z = -0.170;

  // request->body_leg_ik.leg_points.front_left_leg.x = 0.115;
  // request->body_leg_ik.leg_points.front_left_leg.y = 0.118;
  // request->body_leg_ik.leg_points.front_left_leg.z = -0.170;

  // request->body_leg_ik.leg_points.back_left_leg.x = -0.115;
  // request->body_leg_ik.leg_points.back_left_leg.y = 0.118;
  // request->body_leg_ik.leg_points.back_left_leg.z = -0.170;

  // request->body_leg_ik.leg_points.back_right_leg.x = -0.115;
  // request->body_leg_ik.leg_points.back_right_leg.y = -0.118;
  // request->body_leg_ik.leg_points.back_right_leg.z = -0.170;


  request->body_leg_ik.reference_link = black_mouth_kinematics::msg::BodyLegIK::FOOT_LINK_AS_REFERENCE;
  request->body_leg_ik.leg_points.front_right_leg.x = 0.0;
  request->body_leg_ik.leg_points.front_right_leg.y = 0.0;
  request->body_leg_ik.leg_points.front_right_leg.z = 0.0;

  request->body_leg_ik.leg_points.front_left_leg.x = 0.0;
  request->body_leg_ik.leg_points.front_left_leg.y = 0.0;
  request->body_leg_ik.leg_points.front_left_leg.z = 0.0;

  request->body_leg_ik.leg_points.back_left_leg.x = 0.0;
  request->body_leg_ik.leg_points.back_left_leg.y = 0.0;
  request->body_leg_ik.leg_points.back_left_leg.z = 0.0;

  request->body_leg_ik.leg_points.back_right_leg.x = 0.0;
  request->body_leg_ik.leg_points.back_right_leg.y = 0.0;
  request->body_leg_ik.leg_points.back_right_leg.z = 0.0;


  // request->body_leg_ik.reference_link = black_mouth_kinematics::msg::BodyLegIK::HIP_LINK_AS_REFERENCE;
  // request->body_leg_ik.leg_points.front_right_leg.x = 0.0;
  // request->body_leg_ik.leg_points.front_right_leg.y = -0.048;
  // request->body_leg_ik.leg_points.front_right_leg.z = -0.170;

  // request->body_leg_ik.leg_points.front_left_leg.x = 0.0;
  // request->body_leg_ik.leg_points.front_left_leg.y = 0.048;
  // request->body_leg_ik.leg_points.front_left_leg.z = -0.170;

  // request->body_leg_ik.leg_points.back_left_leg.x = 0.0;
  // request->body_leg_ik.leg_points.back_left_leg.y = 0.048;
  // request->body_leg_ik.leg_points.back_left_leg.z = -0.170;

  // request->body_leg_ik.leg_points.back_right_leg.x = 0.0;
  // request->body_leg_ik.leg_points.back_right_leg.y = -0.048;
  // request->body_leg_ik.leg_points.back_right_leg.z = -0.170;


  while(!client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto start = std::chrono::steady_clock::now();

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    black_mouth_kinematics::msg::LegJoints front_right = result.get()->leg_joints.front_right_leg;
    black_mouth_kinematics::msg::LegJoints front_left  = result.get()->leg_joints.front_left_leg;
    black_mouth_kinematics::msg::LegJoints back_left   = result.get()->leg_joints.back_left_leg;
    black_mouth_kinematics::msg::LegJoints back_right  = result.get()->leg_joints.back_right_leg;
    RCLCPP_INFO(rclcpp::get_logger("ik_client"), "Leg joints computed: \n \
                  Front Right Leg: %.2f, %.2f. %.2f \n \
                  Front Left Leg: %.2f, %.2f. %.2f \n \
                  back Left Leg: %.2f, %.2f. %.2f \n \
                  back Right Leg: %.2f, %.2f. %.2f",
                  front_right.hip_roll_joint, front_right.hip_pitch_joint, front_right.elbow_joint,
                  front_left.hip_roll_joint,  front_left.hip_pitch_joint,  front_left.elbow_joint,
                  back_left.hip_roll_joint,   back_left.hip_pitch_joint,   back_left.elbow_joint,
                  back_right.hip_roll_joint,  back_right.hip_pitch_joint,  back_right.elbow_joint);
    
    auto end = std::chrono::steady_clock::now();
    std::cout << "Elapsed time in microseconds: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
              << " µs" << std::endl;
  }
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service compute_inverse_kinematics");


  rclcpp::shutdown();
  return 0;

}
