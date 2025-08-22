#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ControllerToggler : public rclcpp::Node
{
public:
  ControllerToggler() : Node("controller_toggler_node")
  {
    // Lista de controladores que queremos gerenciar.
    _controller_names = {
      "front_left_joint_trajectory_controller",
      "front_right_joint_trajectory_controller",
      "back_left_joint_trajectory_controller",
      "back_right_joint_trajectory_controller"
    };

    // Este é o cliente que vai chamar o serviço do controller_manager para ligar/desligar os controladores.
    _switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    // Este é o serviço que nosso nó oferece. Ele será chamado pelo terminal.
    _toggle_service = this->create_service<std_srvs::srv::SetBool>(
      "/toggle_controllers",
      std::bind(&ControllerToggler::toggle_controllers_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Serviço /toggle_controllers pronto para receber comandos.");
  }

private:
  // Esta é a função que é executada quando o serviço /toggle_controllers é chamado.
  void toggle_controllers_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data == true) // Se o comando for para LIGAR
    {
      RCLCPP_INFO(this->get_logger(), "Recebido pedido para ATIVAR os controladores...");
      auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      switch_request->activate_controllers = _controller_names;
      switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
      switch_request->activate_asap = true;

      auto result = _switch_controller_client->async_send_request(switch_request);

      // Espera pelo resultado
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 1s) == rclcpp::FutureReturnCode::SUCCESS)
      {
        if(result.get()->ok)
        {
          RCLCPP_INFO(this->get_logger(), "Controladores ativados com sucesso!");
          response->success = true;
          response->message = "Controllers activated successfully.";
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Falha ao ativar controladores.");
          response->success = false;
          response->message = "Failed to activate controllers.";
        }
      } 
      else 
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout ao chamar o serviço switch_controller.");
        response->success = false;
        response->message = "Service call to switch_controller timed out.";
      }
    }
    else // Se o comando for para DESLIGAR
    {
      RCLCPP_INFO(this->get_logger(), "Recebido pedido para DESATIVAR os controladores...");
      auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      switch_request->deactivate_controllers = _controller_names;
      switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

      auto result = _switch_controller_client->async_send_request(switch_request);

      // Espera pelo resultado
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 1s) == rclcpp::FutureReturnCode::SUCCESS)
      {
        if(result.get()->ok)
        {
          RCLCPP_INFO(this->get_logger(), "Controladores desativados com sucesso!");
          response->success = true;
          response->message = "Controllers deactivated successfully.";
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Falha ao desativar controladores.");
          response->success = false;
          response->message = "Failed to deactivate controllers.";
        }
      } 
      else 
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout ao chamar o serviço switch_controller.");
        response->success = false;
        response->message = "Service call to switch_controller timed out.";
      }
    }
    // NOTA: A lógica para ativar/desativar a "hardware interface" foi omitida por simplicidade,
    // mas pode ser adicionada aqui seguindo o mesmo padrão, criando um cliente para o serviço
    // /controller_manager/set_hardware_component_state.
  }

  // Declaração dos membros da classe
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