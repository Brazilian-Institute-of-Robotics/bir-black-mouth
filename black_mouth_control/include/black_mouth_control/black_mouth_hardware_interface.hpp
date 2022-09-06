#ifndef BLACK_MOUTH_HARDWARE_INTERFACE_HPP_
#define BLACK_MOUTH_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table addresses
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

#define ADDR_RETURN_DELAY_TYPE 9
#define ADDR_DRIVE_TYPE 10
#define ADDR_OPERATING_MODE 11
#define ADDR_MAX_POSITION_LIMIT 48
#define ADDR_MIN_POSITION_LIMIT 52
#define ADDR_POSITION_P_GAIN 84
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_D_GAIN 80

#define LEN_ADDR_TORQUE_ENABLE 1
#define LEN_ADDR_GOAL_POSITION 4
#define LEN_ADDR_PRESENT_POSITION 4

// Protocol version
#define PROTOCOL_VERSION 2.0

namespace black_mouth_control {

struct BMJointInfo {
    // Data for dynamixel's EEPROM area
    int8_t id{0};
    int8_t drive_mode{0};
    int32_t min_pos_limit{0};
    int32_t max_pos_limit{4095};
    int32_t home_angle{2048};

    // Data for dynamixel's RAM area
    int16_t kp_gain{3000};
    int16_t ki_gain{0};
    int16_t kd_gain{500};
    uint8_t write_goal_position[LEN_ADDR_GOAL_POSITION] = {0, 0, 0, 0};
    int32_t present_position{0};
    int32_t goal_position = home_angle;
    // TODO Check feedforward gains

    // Aux convert functions
    std::function<int32_t()> write_convert_func = [&]() -> int32_t {return 0;};
    std::function<double()> read_convert_func = [&]() -> double {return 0.0;};

    // ros2_control interfaces
    double command;
    double state;
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
    // Utils
    bool switch_dynamixel_torque(bool on = true);
    bool check_comm_result(int dxl_comm_result, uint8_t dxl_error);
    double read_convert(int32_t present_pos, int32_t home_pos);
    int32_t write_convert(double command, int32_t home_pos);

    // Hardware parameters
    uint8_t baud_rate_;
    uint8_t return_delay_type_;
    std::string usb_port_;
    std::vector<BMJointInfo> hw_joints_;

    // Dynamixel communication interfaces
    dynamixel::PacketHandler* packet_handler_ = nullptr;
    dynamixel::PortHandler* port_handler_ = nullptr;
    dynamixel::GroupSyncWrite* switchTorqueSyncWrite_ =
        nullptr;  // GroupSyncWrite to switch motor Torque
    dynamixel::GroupSyncWrite* goalPositionSyncWrite_ =
        nullptr;  // GroupSyncWrite to set goal position
    dynamixel::GroupSyncRead* presentPositionSyncRead_ =
        nullptr;  // GroupSyncRead to get present position

    // Dynamixel communication parameters
    int dxl_comm_result_ = COMM_TX_FAIL;
    uint8_t dxl_error_ = 0;
};

}  // namespace black_mouth_control

#endif