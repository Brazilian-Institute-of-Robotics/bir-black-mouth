#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/srv/inv_kinematics.hpp"

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
  request->body_position.x = 1.0;
  request->body_position.y = 2.0;
  request->body_position.z = 3.0;
  request->body_rotation.x = 1.0;
  request->body_rotation.y = 2.0;
  request->body_rotation.z = 3.0;


  while(!client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    black_mouth_kinematics::msg::LegJoints front_right = result.get()->front_right_leg;
    black_mouth_kinematics::msg::LegJoints front_left  = result.get()->front_left_leg;
    black_mouth_kinematics::msg::LegJoints back_left   = result.get()->back_left_leg;
    black_mouth_kinematics::msg::LegJoints back_right  = result.get()->back_right_leg;
    RCLCPP_INFO(rclcpp::get_logger("ik_client"), "Leg joints computed: \n \
                  Front Right Leg: %.2f, %.2f. %.2f \n \
                  Front Left Leg: %.2f, %.2f. %.2f \n \
                  back Left Leg: %.2f, %.2f. %.2f \n \
                  back Right Leg: %.2f, %.2f. %.2f",
                  front_right.hip_roll_joint, front_right.hip_pitch_joint, front_right.elbow_joint,
                  front_left.hip_roll_joint,  front_left.hip_pitch_joint,  front_left.elbow_joint,
                  back_left.hip_roll_joint,   back_left.hip_pitch_joint,   back_left.elbow_joint,
                  back_right.hip_roll_joint,  back_right.hip_pitch_joint,  back_right.elbow_joint);
  }
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service compute_inverse_kinematics");

  rclcpp::shutdown();
  return 0;

}
