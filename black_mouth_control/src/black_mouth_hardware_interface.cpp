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

    port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
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
    dxl_comm_result_ = port_handler_->setBaudRate(baud_rate_);
    if (dxl_comm_result_ == false) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to set the baudrate!");
        return hardware_interface::CallbackReturn::FAILURE;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                    "Succeeded to set the baudrate.");
    }

    // Set Dynamixel group communication interfaces
    switchTorqueSyncWrite_ =
        (dynamixel::GroupSyncWrite*)(new dynamixel::GroupSyncWrite(
            port_handler_, packet_handler_, ADDR_TORQUE_ENABLE,
            LEN_ADDR_TORQUE_ENABLE));
    goalPositionSyncWrite_ =
        (dynamixel::GroupSyncWrite*)(new dynamixel::GroupSyncWrite(
            port_handler_, packet_handler_, ADDR_GOAL_POSITION,
            LEN_ADDR_GOAL_POSITION));
    presentPositionSyncRead_ =
        (dynamixel::GroupSyncRead*)(new dynamixel::GroupSyncRead(
            port_handler_, packet_handler_, ADDR_PRESENT_POSITION,
            LEN_ADDR_PRESENT_POSITION));

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

        // Add dynamixel to read present_position group
        presentPositionSyncRead_->addParam(hw_joints_[i].id);

        // Add dynamixel to write goal position group
        hw_joints_[i].goal_position =
            write_convert(hw_joints_[i].command, hw_joints_[i].home_angle);

        hw_joints_[i].write_goal_position[0] =
            DXL_LOBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
        hw_joints_[i].write_goal_position[1] =
            DXL_HIBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
        hw_joints_[i].write_goal_position[2] =
            DXL_LOBYTE(DXL_HIWORD(hw_joints_[i].goal_position));
        hw_joints_[i].write_goal_position[3] =
            DXL_HIBYTE(DXL_HIWORD(hw_joints_[i].goal_position));

        goalPositionSyncWrite_->addParam(hw_joints_[i].id,
                                         hw_joints_[i].write_goal_position);
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

    // TURN ON TORQUE
    dxl_comm_result_ = switch_dynamixel_torque(true);
    if (dxl_comm_result_ != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to TURN ON torque");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "*** TORQUE IS ON ***");

    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BlackMouthHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                "Deactivating ...please wait...");

    // TURN OFF TORQUE
    // dxl_comm_result_ = switch_dynamixel_torque(false);
    // if (dxl_comm_result_ != COMM_SUCCESS) {
    //     RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
    //                  "Failed to TURN OFF torque");
    //     return hardware_interface::CallbackReturn::FAILURE;
    // }

    // REBOOT Motor
    for (uint i = 0; i < hw_joints_.size(); i++) {
        dxl_comm_result_ = packet_handler_->reboot(port_handler_, hw_joints_[i].id, &dxl_error_);

        if (dxl_comm_result_ != COMM_SUCCESS) {
                RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"), 
                    "TxRxError: ID %d: %s\n",
                    hw_joints_[i].id, packet_handler_->getTxRxResult(dxl_comm_result_));
        }
        else if (dxl_error_ != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"), 
                "DxlError: ID %d: %s\n", hw_joints_[i].id,
                packet_handler_->getRxPacketError(dxl_error_));
        }

    }

    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Finished rebooting motors");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BlackMouthHW::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    // auto start = std::chrono::steady_clock::now();
    (void)period;
    (void)time;

    // Read data from dynamixels
    dxl_comm_result_ = presentPositionSyncRead_->txRxPacket();
    if (dxl_comm_result_ != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to read from Dynamixel");
        return hardware_interface::return_type::ERROR;
    }

    for (uint i = 0; i < hw_joints_.size(); i++) {
        // Get data from each motor
        // presentPositionSyncRead_->isAvailable(hw_joints_[i].id,
        //                                       ADDR_PRESENT_POSITION,
        //                                       LEN_ADDR_PRESENT_POSITION);
        hw_joints_[i].present_position = presentPositionSyncRead_->getData(
            hw_joints_[i].id, ADDR_PRESENT_POSITION,
            LEN_ADDR_PRESENT_POSITION);

        // Convert byte data to radians
        if (hw_joints_[i].id % 10 != 3) {
            hw_joints_[i].state = read_convert(hw_joints_[i].present_position,
                                               hw_joints_[i].home_angle);
        } else {
            double motor_angle = read_convert(hw_joints_[i].present_position,
                                              hw_joints_[i].home_angle);
            hw_joints_[i].state =
                1.12283214 * motor_angle -
                0.01613196;  // Convert motor angle to leg angle
        }
    }

   //TODO DEBUG
    // uint16_t present_load = 0;
    // packet_handler_->read2ByteTxRx(port_handler_, 33, 126, &present_load);
    // int16_t converted_present_load =
    //     (present_load > 10000) ? present_load - 65535 : present_load;
    // RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Load 33: %d", converted_present_load);
    
    // present_load = 0;
    // packet_handler_->read2ByteTxRx(port_handler_, 23, 126, &present_load);
    // converted_present_load =
    //     (present_load > 10000) ? present_load - 65535 : present_load;
    // RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"), "Load 23: %d\n", converted_present_load);
    
    // auto end = std::chrono::steady_clock::now();
    // std::cout << "READ: "
    //           << std::chrono::duration_cast<std::chrono::microseconds>(end -
    //                                                                    start)
    //                  .count()
    //           << " µs" << std::endl;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BlackMouthHW::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    // auto start = std::chrono::steady_clock::now();

    (void)period;
    (void)time;

    for (uint i = 0; i < hw_joints_.size(); i++) {
        // Convert command to dynamixel byte data
        if (hw_joints_[i].id % 10 != 3) {
            hw_joints_[i].goal_position =
                write_convert(hw_joints_[i].command, hw_joints_[i].home_angle);
        } else {
            double motor_angle =
                0.89058151 * hw_joints_[i].command +
                0.01436683;  // Convert leg angle to motor angle
            hw_joints_[i].goal_position =
                write_convert(motor_angle, hw_joints_[i].home_angle);
        }

        // Apply goal position limits
        if (hw_joints_[i].goal_position > hw_joints_[i].max_pos_limit) {
            hw_joints_[i].goal_position = hw_joints_[i].max_pos_limit;
        } else if (hw_joints_[i].goal_position < hw_joints_[i].min_pos_limit) {
            hw_joints_[i].goal_position = hw_joints_[i].min_pos_limit;
        }

        hw_joints_[i].write_goal_position[0] =
            DXL_LOBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
        hw_joints_[i].write_goal_position[1] =
            DXL_HIBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
        hw_joints_[i].write_goal_position[2] =
            DXL_LOBYTE(DXL_HIWORD(hw_joints_[i].goal_position));
        hw_joints_[i].write_goal_position[3] =
            DXL_HIBYTE(DXL_HIWORD(hw_joints_[i].goal_position));

        goalPositionSyncWrite_->changeParam(hw_joints_[i].id,
                                            hw_joints_[i].write_goal_position);
    }

    // std::cout << "WRITE: " << info_.joints[1].name.c_str() << "  "
    //            << hw_joints_[1].command << "\n";

    dxl_comm_result_ = goalPositionSyncWrite_->txPacket();
    if (dxl_comm_result_ != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to write to Dynamixel");
        return hardware_interface::return_type::ERROR;
    }

    // auto end = std::chrono::steady_clock::now();
    // std::cout << "WRITE: "
    //           << std::chrono::duration_cast<std::chrono::microseconds>(end -
    //                                                                    start)
    //                  .count()
    //           << " µs" << std::endl;

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

double BlackMouthHW::read_convert(int32_t present_pos, int32_t home_pos) {
    return (present_pos - home_pos) * (M_PI * 0.088) / 180;
}

int32_t BlackMouthHW::write_convert(double command, int32_t home_pos) {
    return (int32_t)(command * 180 / (M_PI * 0.088) + home_pos);
}

}  // namespace black_mouth_control

PLUGINLIB_EXPORT_CLASS(black_mouth_control::BlackMouthHW,
                       hardware_interface::SystemInterface)
