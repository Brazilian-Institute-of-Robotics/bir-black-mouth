#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include <memory>
#include <string>
#include <vector>
#include <future>

using namespace std::chrono_literals;

class ControllerToggler : public rclcpp::Node
{
public:
  ControllerToggler() : Node("controller_toggler_node")
  {
    _controller_names = {
      "front_left_joint_trajectory_controller",
      "front_right_joint_trajectory_controller",
      "back_left_joint_trajectory_controller",
      "back_right_joint_trajectory_controller"
    };

    _switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    _toggle_service = this->create_service<std_srvs::srv::SetBool>(
      "/toggle_controllers",
      std::bind(&ControllerToggler::toggle_controllers_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "toggle_controllers ready.");
  }

private:
  void toggle_controllers_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    
    if (request->data == true)
    {
      RCLCPP_INFO(this->get_logger(), "Activating Controllers...");
      switch_request->activate_controllers = _controller_names;
      switch_request->activate_asap = true;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Deactivating Controllers...");
      switch_request->deactivate_controllers = _controller_names;
    }

    // Espera o serviço do controller_manager estar disponível
    if (!_switch_controller_client->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Controller manager service not available.");
      response->success = false;
      response->message = "Controller manager service not available.";
      return;
    }

    auto future_result = _switch_controller_client->async_send_request(switch_request);
    
    // CORREÇÃO: Usa wait_for para esperar o resultado, evitando o conflito de spin
    std::future_status status = future_result.wait_for(2s);

    if (status == std::future_status::ready)
    {
      if (future_result.get()->ok)
      {
        std::string action = request->data ? "Activated" : "Deactivated";
        RCLCPP_INFO(this->get_logger(), "Controllers  %s successfully activated!", action.c_str());
        response->success = true;
        response->message = "Controllers switched successfully.";
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers.");
        response->success = false;
        response->message = "Failed to switch controllers.";
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout switch_controller.");
      response->success = false;
      response->message = "Service call to switch_controller timed out.";
    }
  }

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _toggle_service;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr _switch_controller_client;
  std::vector<std::string> _controller_names;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerToggler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}