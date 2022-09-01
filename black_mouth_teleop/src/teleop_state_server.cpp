#include "rclcpp/rclcpp.hpp"
#include "black_mouth_teleop/msg/teleop_state.hpp"
#include "black_mouth_teleop/srv/set_teleop_state.hpp"
#include "black_mouth_teleop/srv/get_teleop_state.hpp"

#include <memory>
#include <cmath>

std::vector<std::string> states = {"INIT", 
                                   "RESTING", 
                                   "BODY_LOCKED", 
                                   "CONTROLLING_BODY", 
                                   "WALKING"};
int current_state;

void setTeleopState(const std::shared_ptr<black_mouth_teleop::srv::SetTeleopState::Request> request,
                          std::shared_ptr<black_mouth_teleop::srv::SetTeleopState::Response> responde)
{
  (void) responde;
  current_state = request->state.state;

  RCLCPP_INFO(rclcpp::get_logger("teleop_state_server"), "Teleop State set to %s", states.at(current_state).c_str());
}

void getTeleopState(const std::shared_ptr<black_mouth_teleop::srv::GetTeleopState::Request> request,
                          std::shared_ptr<black_mouth_teleop::srv::GetTeleopState::Response> responde)
{
  (void) request;
  responde->state.state = current_state;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("teleop_state_server");
    
  rclcpp::Service<black_mouth_teleop::srv::SetTeleopState>::SharedPtr set_service = 
    node->create_service<black_mouth_teleop::srv::SetTeleopState>("set_teleop_state", &setTeleopState);

  rclcpp::Service<black_mouth_teleop::srv::GetTeleopState>::SharedPtr get_service = 
    node->create_service<black_mouth_teleop::srv::GetTeleopState>("get_teleop_state", &getTeleopState);

  RCLCPP_INFO(rclcpp::get_logger("teleop_state_server"), "Teleop State Server initialized");

  rclcpp::spin(node);
  rclcpp::shutdown();

}
