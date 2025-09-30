#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include <memory>
#include <string>
#include <vector>
#include <future>
#include "rclcpp/executors.hpp"

using namespace std::chrono_literals;

class ControllerToggler : public rclcpp::Node
{
public:
  ControllerToggler() : Node("controller_toggler_node")
  {
    _callback_group = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    _controller_names = {
      "front_left_joint_trajectory_controller",
      "front_right_joint_trajectory_controller",
      "back_left_joint_trajectory_controller",
      "back_right_joint_trajectory_controller"
    };
    
    _hardware_component_name = "CaramelSystem";

    _switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    _set_hw_state_client = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>("/controller_manager/set_hardware_component_state");
    _default_pose_publisher = this->create_publisher<std_msgs::msg::Empty>("/cmd_default_pose", 10);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service =
      this->create_service<std_srvs::srv::SetBool>(
        "/toggle_controllers",
        std::bind(&ControllerToggler::toggle_controllers_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        _callback_group);
    _toggle_service = service;

    RCLCPP_INFO(this->get_logger(), "Serviço /toggle_controllers pronto para ligar e desligar o robô.");
  }

private:
  void toggle_controllers_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data == true)
    {
      RCLCPP_INFO(this->get_logger(), "INICIANDO: Sequência de ativação do robô...");

      if (!set_hardware_state(true)) {
          response->success = false;
          response->message = "Falha ao ATIVAR a hardware interface.";
          RCLCPP_ERROR(this->get_logger(), response->message.c_str());
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Passo 1/3: Hardware Interface ativada.");

      if (!switch_controllers(true)) {
          response->success = false;
          response->message = "Falha ao ATIVAR os controladores das pernas.";
          RCLCPP_ERROR(this->get_logger(), response->message.c_str());
          set_hardware_state(false); 
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Passo 2/3: Controladores ativados.");
      
      // Este é o passo crucial que estava faltando nos nossos testes anteriores
      RCLCPP_INFO(this->get_logger(), "Passo 3/3: Enviando comando para a posição padrão...");
      _default_pose_publisher->publish(std_msgs::msg::Empty());
      rclcpp::sleep_for(3s); 
      
      RCLCPP_INFO(this->get_logger(), "ROBÔ PRONTO: Sequência de ativação concluída.");
      response->success = true;
      response->message = "Robô ativado e na posição padrão.";
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "INICIANDO: Sequência de desligamento do robô...");

      if (!switch_controllers(false)) {
          response->success = false;
          response->message = "Falha ao DESATIVAR os controladores das pernas.";
          RCLCPP_ERROR(this->get_logger(), response->message.c_str());
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Passo 1/2: Controladores desativados.");

      if (!set_hardware_state(false)) {
          response->success = false;
          response->message = "Falha ao DESATIVAR a hardware interface.";
          RCLCPP_ERROR(this->get_logger(), response->message.c_str());
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Passo 2/2: Hardware Interface desativada.");
      
      RCLCPP_INFO(this->get_logger(), "ROBÔ DESLIGADO: Sequência de desligamento concluída.");
      response->success = true;
      response->message = "Robô desligado com segurança.";
    }
  }

  bool set_hardware_state(bool activate)
  {
    auto request = std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
    request->name = _hardware_component_name;
    request->target_state.id = activate ? 3 : 1;

    if (!_set_hw_state_client->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Serviço /controller_manager/set_hardware_component_state não disponível.");
      return false;
    }

    auto future = _set_hw_state_client->async_send_request(request);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get()->ok;
    }
    RCLCPP_ERROR(this->get_logger(), "Timeout ao chamar o serviço set_hardware_component_state.");
    return false;
  }

  bool switch_controllers(bool activate)
  {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    if (activate) {
      request->activate_controllers = _controller_names;
    } else {
      request->deactivate_controllers = _controller_names;
    }
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->activate_asap = true;

    if (!_switch_controller_client->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Serviço /controller_manager/switch_controller não disponível.");
      return false;
    }

    auto future = _switch_controller_client->async_send_request(request);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get()->ok;
    }
    RCLCPP_ERROR(this->get_logger(), "Timeout ao chamar o serviço switch_controller.");
    return false;
  }

  rclcpp::CallbackGroup::SharedPtr _callback_group;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _toggle_service;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr _switch_controller_client;
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr _set_hw_state_client;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _default_pose_publisher;
  std::vector<std::string> _controller_names;
  std::string _hardware_component_name;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerToggler>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}