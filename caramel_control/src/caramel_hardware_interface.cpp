#include "caramel_control/caramel_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <algorithm>

// Definição de Endereços
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_DRIVE_TYPE 10
#define ADDR_MAX_POSITION_LIMIT 48
#define ADDR_MIN_POSITION_LIMIT 52
#define ADDR_RETURN_DELAY_TYPE 9
#define ADDR_POSITION_P_GAIN 84
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_D_GAIN 80
#define LEN_ADDR_PRESENT_POSITION 4
#define LEN_ADDR_TORQUE_ENABLE 1
#define PROTOCOL_VERSION 2.0

namespace caramel_control {

// --- FUNÇÕES AUXILIARES ---

double normalize_angle(double angle) {
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0) return result + M_PI;
    return result - M_PI;
}

int16_t nm_to_current_raw(double torque_nm, double current_unit_ma) {
    double kt = 1.778; 
    if (torque_nm > 3.0) torque_nm = 3.0;
    if (torque_nm < -3.0) torque_nm = -3.0;

    double target_current_a = torque_nm / kt;
    double current_unit_a = current_unit_ma / 1000.0;
    int32_t raw_value = (int32_t)(target_current_a / current_unit_a);
    
    int32_t MAX_RAW = 1193;
    if (raw_value > MAX_RAW) raw_value = MAX_RAW;
    if (raw_value < -MAX_RAW) raw_value = -MAX_RAW;

    return (int16_t)raw_value;
}

hardware_interface::CallbackReturn CaramelHW::on_init(
    const hardware_interface::HardwareInfo& info) {
    
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("CaramelHW"), "Joint '%s' expected 1 Position Command Interface.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("CaramelHW"), "Joint '%s' expected 1 Position State Interface.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    baud_rate_ = stoi(info_.hardware_parameters["baud_rate"]);
    usb_port_ = info_.hardware_parameters["usb_port"];
    return_delay_type_ = stoi(info_.hardware_parameters["return_delay_type"]);

    pd_control_enabled_ = false;
    if (info_.hardware_parameters.count("pd_control") > 0) {
        if (info_.hardware_parameters.at("pd_control") == "true") {
            pd_control_enabled_ = true;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("CaramelHW"), 
        "Init: Baud %d | Port %s | PD CONTROL ENABLED: %s", 
        baud_rate_, usb_port_.c_str(), pd_control_enabled_ ? "TRUE" : "FALSE");

    hw_joints_.resize(info_.joints.size());
    for (uint i = 0; i < hw_joints_.size(); i++) {
        hw_joints_[i].name = info_.joints[i].name;
        hw_joints_[i].id = stoi(info_.joints[i].parameters.at("id"));
        hw_joints_[i].drive_mode = stoi(info_.joints[i].parameters.at("drive_mode"));
        hw_joints_[i].home_angle = stoi(info_.joints[i].parameters.at("home_angle"));
        hw_joints_[i].min_pos_limit = stoi(info_.joints[i].parameters.at("min_pos_limit"));
        hw_joints_[i].max_pos_limit = stoi(info_.joints[i].parameters.at("max_pos_limit"));
        hw_joints_[i].kp_gain = stoi(info_.joints[i].parameters.at("kp_gain"));
        hw_joints_[i].ki_gain = stoi(info_.joints[i].parameters.at("ki_gain"));
        hw_joints_[i].kd_gain = stoi(info_.joints[i].parameters.at("kd_gain"));

        if (pd_control_enabled_) {
             if (info_.joints[i].parameters.count("ctrl_kp"))
                hw_joints_[i].ctrl_kp = std::stod(info_.joints[i].parameters.at("ctrl_kp"));
             else 
                RCLCPP_WARN(rclcpp::get_logger("CaramelHW"), "Joint %s missing ctrl_kp!", hw_joints_[i].name.c_str());

             if (info_.joints[i].parameters.count("ctrl_kd"))
                hw_joints_[i].ctrl_kd = std::stod(info_.joints[i].parameters.at("ctrl_kd"));
             
             // (Leitura de ff_torque removida daqui)
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CaramelHW::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("CaramelHW"), "Configuring hardware...");
    (void)previous_state;

    port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_handler_->openPort() || !port_handler_->setBaudRate(baud_rate_)) {
        RCLCPP_ERROR(rclcpp::get_logger("CaramelHW"), "Failed to open port or set baudrate!");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    presentPositionSyncRead_ = (dynamixel::GroupSyncRead*)(new dynamixel::GroupSyncRead(
            port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_ADDR_PRESENT_POSITION));

    switchTorqueSyncWrite_ = (dynamixel::GroupSyncWrite*)(new dynamixel::GroupSyncWrite(
            port_handler_, packet_handler_, ADDR_TORQUE_ENABLE, LEN_ADDR_TORQUE_ENABLE));

    if (pd_control_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("CaramelHW"), "Configuring SyncWrite for CURRENT (Addr %d)", ADDR_GOAL_CURRENT);
        goalPositionSyncWrite_ = (dynamixel::GroupSyncWrite*)(new dynamixel::GroupSyncWrite(
            port_handler_, packet_handler_, ADDR_GOAL_CURRENT, 2)); 
    } else {
        RCLCPP_INFO(rclcpp::get_logger("CaramelHW"), "Configuring SyncWrite for POSITION (Addr %d)", ADDR_GOAL_POSITION);
        goalPositionSyncWrite_ = (dynamixel::GroupSyncWrite*)(new dynamixel::GroupSyncWrite(
            port_handler_, packet_handler_, ADDR_GOAL_POSITION, 4));
    }

    switch_dynamixel_torque(false);

    for (uint i = 0; i < hw_joints_.size(); i++) {
        hw_joints_[i].state = 0;
        hw_joints_[i].command = 0;

        packet_handler_->write1ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_RETURN_DELAY_TYPE, return_delay_type_);
        packet_handler_->write1ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_DRIVE_TYPE, hw_joints_[i].drive_mode);
        packet_handler_->write4ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_MAX_POSITION_LIMIT, hw_joints_[i].max_pos_limit);
        packet_handler_->write4ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_MIN_POSITION_LIMIT, hw_joints_[i].min_pos_limit);
        packet_handler_->write2ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_POSITION_P_GAIN, hw_joints_[i].kp_gain);
        packet_handler_->write2ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_POSITION_I_GAIN, hw_joints_[i].ki_gain);
        packet_handler_->write2ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_POSITION_D_GAIN, hw_joints_[i].kd_gain);

        if (pd_control_enabled_) {
            // Modo 0: Current Control
            packet_handler_->write1ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_OPERATING_MODE, 0);
            hw_joints_[i].velocity = 0.0;
            hw_joints_[i].prev_velocity = 0.0;
            uint8_t d[2] = {0, 0};
            goalPositionSyncWrite_->addParam(hw_joints_[i].id, d);
        } else {
            // Modo 3: Position Control
            packet_handler_->write1ByteTxRx(port_handler_, hw_joints_[i].id, ADDR_OPERATING_MODE, 3);
            hw_joints_[i].goal_position = write_convert(0.0, hw_joints_[i].home_angle);
            uint8_t d[4];
            d[0] = DXL_LOBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
            d[1] = DXL_HIBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
            d[2] = DXL_LOBYTE(DXL_HIWORD(hw_joints_[i].goal_position));
            d[3] = DXL_HIBYTE(DXL_HIWORD(hw_joints_[i].goal_position));
            goalPositionSyncWrite_->addParam(hw_joints_[i].id, d);
        }
        presentPositionSyncRead_->addParam(hw_joints_[i].id);
    }

    RCLCPP_INFO(rclcpp::get_logger("CaramelHW"), "Hardware Configured Successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CaramelHW::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;

    const std::lock_guard<std::mutex> lock(mutex_);

    dxl_comm_result_ = presentPositionSyncRead_->txRxPacket();
    if (dxl_comm_result_ != COMM_SUCCESS) {
        // Mantive a proteção de crash, pois sem ela o robô desliga com qualquer ruído
        RCLCPP_WARN(rclcpp::get_logger("CaramelHW"), "Pacote perdido! Ignorando este ciclo...");
        return hardware_interface::return_type::OK; 
    }

    double dt = period.seconds();
    if (dt < 0.0001) dt = 0.0001;

    for (uint i = 0; i < hw_joints_.size(); i++) {
        hw_joints_[i].present_position = presentPositionSyncRead_->getData(
            hw_joints_[i].id, ADDR_PRESENT_POSITION, LEN_ADDR_PRESENT_POSITION);

        double current_pos_rad = 0.0;
        if (hw_joints_[i].id % 10 != 3) {
            current_pos_rad = read_convert(hw_joints_[i].present_position, hw_joints_[i].home_angle);
        } else {
            double motor_angle = read_convert(hw_joints_[i].present_position, hw_joints_[i].home_angle);
            current_pos_rad = 1.12283214 * motor_angle - 0.01613196;
        }

        if (pd_control_enabled_) {
            double diff = normalize_angle(current_pos_rad - hw_joints_[i].prev_state);
            double qd_raw = diff / dt;
            hw_joints_[i].velocity = (ALPHA * hw_joints_[i].prev_velocity) + ((1.0 - ALPHA) * qd_raw);
            hw_joints_[i].prev_state = current_pos_rad;
            hw_joints_[i].prev_velocity = hw_joints_[i].velocity;
        }
        hw_joints_[i].state = current_pos_rad;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CaramelHW::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time; (void)period;

    const std::lock_guard<std::mutex> lock(mutex_);

    for (uint i = 0; i < hw_joints_.size(); i++) {
        
        if (pd_control_enabled_) {
            double q_des = hw_joints_[i].command;
            double q_curr = hw_joints_[i].state;
            double qd_curr = hw_joints_[i].velocity;

            double error_p = normalize_angle(q_des - q_curr);
            double error_d = 0.0 - qd_curr; 

            // --- Lógica PD SIMPLES (Sem FF Torque) ---
            double torque = 0.0;
            if (std::abs(error_p) >= DEADZONE_RAD) {
                torque = (hw_joints_[i].ctrl_kp * error_p) + (hw_joints_[i].ctrl_kd * error_d);
            }

            int16_t raw_current = nm_to_current_raw(torque, CURRENT_UNIT_MA);

            hw_joints_[i].write_goal_current[0] = DXL_LOBYTE(raw_current);
            hw_joints_[i].write_goal_current[1] = DXL_HIBYTE(raw_current);
            goalPositionSyncWrite_->changeParam(hw_joints_[i].id, hw_joints_[i].write_goal_current);

        } else {
            // Modo Posição Antigo
            if (hw_joints_[i].id % 10 != 3) {
                hw_joints_[i].goal_position = write_convert(hw_joints_[i].command, hw_joints_[i].home_angle);
            } else {
                double motor_angle = 0.89058151 * hw_joints_[i].command + 0.01436683;
                hw_joints_[i].goal_position = write_convert(motor_angle, hw_joints_[i].home_angle);
            }

            if (hw_joints_[i].goal_position > hw_joints_[i].max_pos_limit) hw_joints_[i].goal_position = hw_joints_[i].max_pos_limit;
            if (hw_joints_[i].goal_position < hw_joints_[i].min_pos_limit) hw_joints_[i].goal_position = hw_joints_[i].min_pos_limit;

            hw_joints_[i].write_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
            hw_joints_[i].write_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(hw_joints_[i].goal_position));
            hw_joints_[i].write_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(hw_joints_[i].goal_position));
            hw_joints_[i].write_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(hw_joints_[i].goal_position));
            goalPositionSyncWrite_->changeParam(hw_joints_[i].id, hw_joints_[i].write_goal_position);
        }
    }

    dxl_comm_result_ = goalPositionSyncWrite_->txPacket();

    if (dxl_comm_result_ != COMM_SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("CaramelHW"), "Falha na escrita! Ignorando...");
        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

// ... Métodos de export_state, export_command, activate e deactivate permanecem iguais ...
// (Para economizar espaço, eles são idênticos ao código anterior)

std::vector<hardware_interface::StateInterface> CaramelHW::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joints_[i].state));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CaramelHW::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joints_[i].command));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn CaramelHW::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("CaramelHW"), "Activating... Torque ON");
    const std::lock_guard<std::mutex> lock(mutex_);
    for (uint i = 0; i < hw_joints_.size(); i++) {
        hw_joints_[i].command = hw_joints_[i].state;
        hw_joints_[i].prev_state = hw_joints_[i].state;
    }
    switch_dynamixel_torque(true);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CaramelHW::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("CaramelHW"), "Deactivating... Rebooting");
    const std::lock_guard<std::mutex> lock(mutex_);
    for (uint i = 0; i < hw_joints_.size(); i++) {
        packet_handler_->reboot(port_handler_, hw_joints_[i].id, &dxl_error_);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool CaramelHW::switch_dynamixel_torque(bool on) {
    if (switchTorqueSyncWrite_ == NULL) return false;
    uint8_t data = on ? 1 : 0;
    switchTorqueSyncWrite_->clearParam();
    for (uint i = 0; i < hw_joints_.size(); i++) {
        switchTorqueSyncWrite_->addParam(hw_joints_[i].id, &data);
    }
    return switchTorqueSyncWrite_->txPacket() == COMM_SUCCESS;
}

bool CaramelHW::check_comm_result(int dxl_comm_result, uint8_t dxl_error) {
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) return false;
    return true;
}

double CaramelHW::read_convert(int32_t present_pos, int32_t home_pos) {
    return (present_pos - home_pos) * (M_PI * 0.088) / 180.0;
}

int32_t CaramelHW::write_convert(double command, int32_t home_pos) {
    return (int32_t)(command * 180.0 / (M_PI * 0.088) + home_pos);
}

} // namespace caramel_control

PLUGINLIB_EXPORT_CLASS(caramel_control::CaramelHW, hardware_interface::SystemInterface)