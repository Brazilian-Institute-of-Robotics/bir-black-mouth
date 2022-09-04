#include "black_mouth_control/black_mouth_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace black_mouth_control {

hardware_interface::CallbackReturn BlackMouthHW::on_init(
    const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Make sure only position command_interface and position state_interface
    // are defined
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BlackMouthHW"),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BlackMouthHW"),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("BlackMouthHW"),
                         "Joint '%s' has %zu state interface. 1 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("BlackMouthHW"),
                         "Joint '%s' have %s state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Get System hardware parameters from URDF
    baud_rate_ = stoi(info_.hardware_parameters["baud_rate"]);
    usb_port_ = info_.hardware_parameters["usb_port"];
    return_delay_type_ = stoi(info_.hardware_parameters["return_delay_type"]);

    // Log hardware info
    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                "Hardware settings:"
                "\n\tbaud_rate = %d"
                "\n\tusb_port = %s"
                "\n\treturn_delay_type = %d",
                baud_rate_, usb_port_.c_str(), return_delay_type_);

    // Get joint parameters
    hw_joints_.resize(info_.joints.size());
    for (uint i = 0; i < hw_joints_.size(); i++) {
        hw_joints_[i].id = stoi(info_.joints[i].parameters.at("id"));
        hw_joints_[i].drive_mode =
            stoi(info_.joints[i].parameters.at("drive_mode"));
        hw_joints_[i].home_angle =
            stoi(info_.joints[i].parameters.at("home_angle"));
        hw_joints_[i].min_pos_limit =
            stoi(info_.joints[i].parameters.at("min_pos_limit"));
        hw_joints_[i].max_pos_limit =
            stoi(info_.joints[i].parameters.at("max_pos_limit"));
        hw_joints_[i].kp_gain = stoi(info_.joints[i].parameters.at("kp_gain"));
        hw_joints_[i].ki_gain = stoi(info_.joints[i].parameters.at("ki_gain"));
        hw_joints_[i].kd_gain = stoi(info_.joints[i].parameters.at("kd_gain"));

        // Log joint info
        RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                    "\n\n"
                    "Joint: %s"
                    "\n\tid = %d"
                    "\n\tdrive_mode = %d"
                    "\n\thome_angle = %d"
                    "\n\tmin_pos_limit = %d"
                    "\n\tmax_pos_limit = %d"
                    "\n\tkp_gain = %d"
                    "\n\tki_gain = %d"
                    "\n\tkd_gain = %d",
                    info_.joints[i].name.c_str(), hw_joints_[i].id,
                    hw_joints_[i].drive_mode, hw_joints_[i].home_angle,
                    hw_joints_[i].min_pos_limit, hw_joints_[i].max_pos_limit,
                    hw_joints_[i].kp_gain, hw_joints_[i].ki_gain,
                    hw_joints_[i].kd_gain);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BlackMouthHW::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Configuring hardware...");

    (void)previous_state;

    port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packet_handler_ =
        dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    dxl_comm_result_ = port_handler_->openPort();
    if (dxl_comm_result_ == false) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to open the port!");
        return hardware_interface::CallbackReturn::FAILURE;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                    "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port
    dxl_comm_result_ = port_handler_->setBaudRate(BAUDRATE);
    if (dxl_comm_result_ == false) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to set the baudrate!");
        return hardware_interface::CallbackReturn::FAILURE;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                    "Succeeded to set the baudrate.");
    }

    // Set Dynamixel group communication interfaces
    *switchTorqueSyncWrite_ =
        dynamixel::GroupSyncWrite(port_handler_, packet_handler_,
                                  ADDR_TORQUE_ENABLE, LEN_ADDR_TORQUE_ENABLE);
    *goalPositionSyncWrite_ =
        dynamixel::GroupSyncWrite(port_handler_, packet_handler_,
                                  ADDR_GOAL_POSITION, LEN_ADDR_GOAL_POSITION);
    *presentPositionSyncRead_ = dynamixel::GroupSyncRead(
        port_handler_, packet_handler_, ADDR_PRESENT_POSITION,
        LEN_ADDR_PRESENT_POSITION);

    // TURN OFF TORQUE
    dxl_comm_result_ = switch_dynamixel_torque(false);
    if (dxl_comm_result_ != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to TURN OFF torque");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    // Configure Dynamixels
    for (uint i = 0; i < hw_joints_.size(); i++) {
        // Reset ros2_control interfaces
        hw_joints_[i].state = 0;
        hw_joints_[i].command = 0;

        // Set motor Return Delay Type
        dxl_comm_result_ = packet_handler_->write1ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_RETURN_DELAY_TYPE,
            return_delay_type_);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        // Set motor Operating Mode to Position Control
        dxl_comm_result_ = packet_handler_->write1ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_OPERATING_MODE, 3);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        // Set motor Drive type
        dxl_comm_result_ = packet_handler_->write1ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_DRIVE_TYPE,
            hw_joints_[i].drive_mode);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        // Set motor max position limit
        dxl_comm_result_ = packet_handler_->write4ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_MAX_POSITION_LIMIT,
            hw_joints_[i].max_pos_limit);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        // Set motor min position limit
        dxl_comm_result_ = packet_handler_->write4ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_MIN_POSITION_LIMIT,
            hw_joints_[i].min_pos_limit);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        // Set motor position P gain
        dxl_comm_result_ = packet_handler_->write2ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_POSITION_P_GAIN,
            hw_joints_[i].kp_gain);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        // Set motor position I gain
        dxl_comm_result_ = packet_handler_->write2ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_POSITION_I_GAIN,
            hw_joints_[i].ki_gain);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        // Set motor position D gain
        dxl_comm_result_ = packet_handler_->write2ByteTxRx(
            port_handler_, hw_joints_[i].id, ADDR_POSITION_D_GAIN,
            hw_joints_[i].kd_gain);
        if (!check_comm_result(dxl_comm_result_, dxl_error_)) {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        //TODO Create convert functions
        // hw_joints_[i].write_convert_func = [&](double command) -> int32_t {
        //     return (int32_t)(command*180/(3.14*0.088) + hw_joints_[i].home_angle);
        // }
        // hw_joints_[i].read_convert_func = [&](uint32_t present_position) -> double {
        //     return (present_position - hw_joints_[i].home_angle)*(3.14*0.088)/180;
        // }

        presentPositionSyncRead_->addParam(hw_joints_[i].id);
        // goalPositionSyncWrite_->addParam(
        //     hw_joints_[i].id,
        //     &hw_joints_[i].write_convert_func(hw_joints_[i].command))
    }

    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BlackMouthHW::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_joints_[i].state));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BlackMouthHW::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_joints_[i].command));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn BlackMouthHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < hw_joints_.size(); i++) {
        hw_joints_[i].command = hw_joints_[i].state;
    }

    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BlackMouthHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                "Deactivating ...please wait...");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BlackMouthHW::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)period;
    (void)time;

    // RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Reading...");

    for (uint i = 0; i < hw_joints_.size(); i++) {
        // Simulate RRBot's movement
        hw_joints_[i].state = 123.;
        // RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
        //             "Got state %.5f for joint %d!", hw_states_[i], i);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BlackMouthHW::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)period;
    (void)time;
    // RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Writing...");

    for (uint i = 0; i < hw_joints_.size(); i++) {
        // Simulate sending commands to the hardware
        // RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
        //             "Got command %.5f for joint %d!", hw_commands_[i], i);
    }

    return hardware_interface::return_type::OK;
}

bool BlackMouthHW::switch_dynamixel_torque(bool on) {
    // Check communication interface existence
    if (switchTorqueSyncWrite_ == NULL) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Dynamixel interfaces were not correctly initialized");
        return false;
    }

    uint8_t turn_on = 1;
    uint8_t turn_off = 0;

    switchTorqueSyncWrite_->clearParam();

    for (uint i = 0; i < hw_joints_.size(); i++) {
        if (on) {
            switchTorqueSyncWrite_->addParam(hw_joints_[i].id, &turn_on);
        } else {
            switchTorqueSyncWrite_->addParam(hw_joints_[i].id, &turn_off);
        }
    }

    return switchTorqueSyncWrite_->txPacket();
}

bool BlackMouthHW::check_comm_result(int dxl_comm_result, uint8_t dxl_error) {
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"), "%s\n",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
    } else if (dxl_error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"), "%s\n",
                     packet_handler_->getTxRxResult(dxl_error));
        return false;
    }
    return true;
}

}  // namespace black_mouth_control

PLUGINLIB_EXPORT_CLASS(black_mouth_control::BlackMouthHW,
                       hardware_interface::SystemInterface)