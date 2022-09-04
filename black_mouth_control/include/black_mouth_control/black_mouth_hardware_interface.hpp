#ifndef BLACK_MOUTH_HARDWARE_INTERFACE_HPP_
#define BLACK_MOUTH_HARDWARE_INTERFACE_HPP_

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vector>

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default setting
#define BAUDRATE 4000000  // Port Baudrate
#define DEVICE_NAME "/dev/ttyUSB0"

namespace black_mouth_control {

struct BMJointInfo {
    // Data for dynamixel's EEPROM area
    int8_t id{0};
    int8_t drive_mode{0};
    int32_t min_pos_limit{0};
    int32_t max_pos_limit{4095};
    int32_t home_angle{2048};

    // Data for dynamixel's RAM area
    int32_t convert_func;
    int16_t kp_gain{3000};
    int16_t ki_gain{0};
    int16_t kd_gain{500};
    // TODO Check feedforward gains
};

class BlackMouthHW : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BlackMouthHW)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    // Hardware parameters
    uint8_t baud_rate_;
    uint8_t return_delay_type_;
    std::string usb_port_;
    std::vector<BMJointInfo> hw_joints_;

    // Store the command and states
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

    // Dynamixel communication parameters
    int dxl_comm_result_ = COMM_TX_FAIL;
};

}  // namespace black_mouth_control

#endif