#ifndef CARAMEL_CONTROL__CARAMEL_HARDWARE_INTERFACE_HPP_
#define CARAMEL_CONTROL__CARAMEL_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace caramel_control {

struct Joint {
    std::string name;
    int id;
    
    // Params de Hardware (Dynamixel EEPROM)
    int drive_mode;
    int home_angle;
    int min_pos_limit;
    int max_pos_limit;
    int kp_gain;
    int ki_gain;
    int kd_gain;

    // --- PARÂMETROS PARA O CONTROLADOR PD (CUSTOM) ---
    double ctrl_kp = 0.0;
    double ctrl_kd = 0.0;
    
    // Estados para cálculo de velocidade (Filtro)
    double prev_state = 0.0;
    double velocity = 0.0;
    double prev_velocity = 0.0;

    // Interface ROS2 Control
    double command = 0.0;
    double state = 0.0;
    
    // Buffers de Escrita
    int32_t goal_position = 0;
    uint8_t write_goal_position[4];

    int16_t goal_current = 0;
    uint8_t write_goal_current[2];

    // Leitura
    int32_t present_position = 0;
};

class CaramelHW : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CaramelHW)

    // --- CORREÇÃO AQUI (ROS 2 Jazzy) ---
    // Antes: const hardware_interface::HardwareInfo& info
    // Agora: const hardware_interface::HardwareComponentInterfaceParams& params
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    bool switch_dynamixel_torque(bool on);
    bool check_comm_result(int dxl_comm_result, uint8_t dxl_error);
    double read_convert(int32_t present_pos, int32_t home_pos);
    int32_t write_convert(double command, int32_t home_pos);

    int baud_rate_;
    std::string usb_port_;
    int return_delay_type_;
    
    bool pd_control_enabled_ = false;

    dynamixel::PortHandler* port_handler_;
    dynamixel::PacketHandler* packet_handler_;
    dynamixel::GroupSyncWrite* switchTorqueSyncWrite_;
    dynamixel::GroupSyncWrite* goalPositionSyncWrite_;
    dynamixel::GroupSyncRead* presentPositionSyncRead_;

    int dxl_comm_result_ = COMM_TX_FAIL;
    uint8_t dxl_error_ = 0;
    
    std::vector<Joint> hw_joints_;
    std::mutex mutex_;

    const double ALPHA = 0.8;
    const double DEADZONE_RAD = 0.017;
    const double CURRENT_UNIT_MA = 2.69;
};

}  // namespace caramel_control

#endif  // CARAMEL_CONTROL__CARAMEL_HARDWARE_INTERFACE_HPP_