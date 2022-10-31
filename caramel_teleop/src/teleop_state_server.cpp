#include "rclcpp/rclcpp.hpp"
#include "caramel_teleop/msg/teleop_state.hpp"
#include "caramel_teleop/srv/set_teleop_state.hpp"
#include "caramel_teleop/srv/get_teleop_state.hpp"

#include <memory>
#include <cmath>

#define ANSI_COLOR_RESET "\x1b[0m"
#define ANSI_COLOR_BLUE  "\x1b[34m"

std::vector<std::string> states = {"INIT", 
                                   "RESTING", 
                                   "BODY_LOCKED", 
                                   "CONTROLLING_BODY", 
                                   "MOVING_BODY", 
                                   "WALKING"};
caramel_teleop::msg::TeleopState current_state;

void setTeleopState(const std::shared_ptr<caramel_teleop::srv::SetTeleopState::Request> request,
                          std::shared_ptr<caramel_teleop::srv::SetTeleopState::Response> responde)
{
  (void) responde;
  current_state = request->state;

  RCLCPP_INFO(rclcpp::get_logger("teleop_state_server"), "State: " ANSI_COLOR_BLUE "\x1b[34m"
 "\33[1m%s\33[0m" ANSI_COLOR_RESET, states.at(current_state.state).c_str());
}

void getTeleopState(const std::shared_ptr<caramel_teleop::srv::GetTeleopState::Request> request,
                          std::shared_ptr<caramel_teleop::srv::GetTeleopState::Response> responde)
{
  (void) request;
  responde->state = current_state;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("teleop_state_server");

  rclcpp::Publisher<caramel_teleop::msg::TeleopState>::SharedPtr state_publisher =
    node->create_publisher<caramel_teleop::msg::TeleopState>("teleop_state", 10);

  rclcpp::Service<caramel_teleop::srv::SetTeleopState>::SharedPtr set_service = 
    node->create_service<caramel_teleop::srv::SetTeleopState>("set_teleop_state", &setTeleopState);

  rclcpp::Service<caramel_teleop::srv::GetTeleopState>::SharedPtr get_service = 
    node->create_service<caramel_teleop::srv::GetTeleopState>("get_teleop_state", &getTeleopState);

  RCLCPP_INFO(rclcpp::get_logger("teleop_state_server"), "Teleop State Server initialized");

  rclcpp::Rate rate(30);
  while(rclcpp::ok())
  {
    state_publisher->publish(current_state);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();

}
