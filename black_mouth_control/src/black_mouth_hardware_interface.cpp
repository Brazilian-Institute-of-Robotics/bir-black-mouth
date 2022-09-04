#include "black_mouth_control/black_mouth_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace black_mouth_control {

// Dynamixel communication
dynamixel::PacketHandler* packet_handler;
dynamixel::PortHandler* port_handler;

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

    // Initialize vectors of joint states and commands
    hw_states_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());

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

    // Reset values always when configuring hardware
    for (uint i = 0; i < hw_states_.size(); i++) {
        hw_states_[i] = 0;
        hw_commands_[i] = 0;
    }

    port_handler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packet_handler =
        dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    dxl_comm_result_ = port_handler->openPort();
    if (dxl_comm_result_ == false) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to open the port!");
        return hardware_interface::CallbackReturn::FAILURE;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                    "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port
    dxl_comm_result_ = port_handler->setBaudRate(BAUDRATE);
    if (dxl_comm_result_ == false) {
        RCLCPP_ERROR(rclcpp::get_logger("BlackMouthHW"),
                     "Failed to set the baudrate!");
        return hardware_interface::CallbackReturn::FAILURE;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                    "Succeeded to set the baudrate.");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BlackMouthHW::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BlackMouthHW::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn BlackMouthHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
                "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < hw_states_.size(); i++) {
        hw_commands_[i] = hw_states_[i];
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

    for (uint i = 0; i < hw_states_.size(); i++) {
        // Simulate RRBot's movement
        hw_states_[i] = 123.;
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

    for (uint i = 0; i < hw_commands_.size(); i++) {
        // Simulate sending commands to the hardware
        // RCLCPP_INFO(rclcpp::get_logger("BlackMouthHW"),
        //             "Got command %.5f for joint %d!", hw_commands_[i], i);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace black_mouth_control

PLUGINLIB_EXPORT_CLASS(black_mouth_control::BlackMouthHW,
                       hardware_interface::SystemInterface)