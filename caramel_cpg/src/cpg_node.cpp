#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <numeric>
#include <algorithm> // Para std::clamp

// Usar apelidos para os tipos de mensagem
using Twist = geometry_msgs::msg::Twist;
using BodyLegIKTrajectory = caramel_kinematics::msg::BodyLegIKTrajectory;
using namespace std::chrono_literals;

// Parâmetros físicos do robô
const double COMPRIMENTO_CORPO = 0.2291;
const double LARGURA_CORPO = 0.140;

class CpgNode : public rclcpp::Node
{
public:
    CpgNode() : Node("cpg_node")
    {
        RCLCPP_INFO(this->get_logger(), "Nó CPG (C++ SEM EIGEN) para Caramel iniciado.");

        // --- Parâmetros do ROS ---
        this->declare_parameter("ganho_amplitude", 20.0);
        this->declare_parameter("forca_acoplamento", 10.0);
        this->declare_parameter("freq_base", 1.0);
        this->declare_parameter("freq_ganho", 2.0);
        this->declare_parameter("freq_max", 4.0);
        this->declare_parameter("ratio_swing_stance", 1.5);
        this->declare_parameter("passo_base", 0.02);
        this->declare_parameter("passo_ganho", 0.2);
        this->declare_parameter("passo_max", 0.10);
        this->declare_parameter("altura_passo", 0.05);
        this->declare_parameter("profundidade_stance", 0.01);

        // --- Estado do CPG (usando std::vector) ---
        phases_.resize(4);
        amplitudes_.assign(4, 0.1);
        for(int i=0; i<4; ++i) {
            phases_[i] = (static_cast<double>(rand()) / RAND_MAX) * 2 * M_PI;
        }

        target_trot_matrix_ = {
            {0, M_PI, M_PI, 0},
            {M_PI, 0, 0, M_PI},
            {M_PI, 0, 0, M_PI},
            {0, M_PI, M_PI, 0}
        };

        // --- Subscriber, Publisher e Timer ---
        sub_ = this->create_subscription<Twist>("/cmd_vel", 10, std::bind(&CpgNode::cmd_vel_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<BodyLegIKTrajectory>("/cmd_ik", 10);
        timer_period_ = 10ms;
        timer_ = this->create_wall_timer(timer_period_, std::bind(&CpgNode::timer_callback, this));
    }

private:
    void cmd_vel_callback(const Twist::SharedPtr msg)
    {
        v_x_ = msg->linear.x;
        v_y_ = msg->linear.y;
        omega_z_ = msg->angular.z;
    }

    void timer_callback()
    {
        step_cpg();
        auto foot_positions = calculate_foot_positions();
        publish_ik_command(foot_positions);
    }

    void step_cpg()
    {
        double dt = std::chrono::duration<double>(timer_period_).count();
        double alpha = this->get_parameter("ganho_amplitude").as_double();
        double forca_acoplamento = this->get_parameter("forca_acoplamento").as_double();
        double freq_base = this->get_parameter("freq_base").as_double();
        double freq_ganho = this->get_parameter("freq_ganho").as_double();
        double freq_max = this->get_parameter("freq_max").as_double();
        double ratio_swing_stance = this->get_parameter("ratio_swing_stance").as_double();

        double velocidade_linear_total = std::sqrt(v_x_ * v_x_ + v_y_ * v_y_);
        double frequencia_hz = freq_base + freq_ganho * velocidade_linear_total;
        double frequencia_stance_hz = std::clamp(frequencia_hz, 0.0, freq_max);
        double frequencia_swing_hz = frequencia_stance_hz * ratio_swing_stance;
        
        double omega_swing = 2 * M_PI * frequencia_swing_hz;
        double omega_stance = 2 * M_PI * frequencia_stance_hz;
        
        double mu = std::clamp(velocidade_linear_total * 2.0, 0.1, 1.0);

        // Atualiza amplitudes (loop manual)
        for (int i = 0; i < 4; ++i) {
            double dot_amplitude = alpha * (mu * mu - amplitudes_[i] * amplitudes_[i]) * amplitudes_[i];
            amplitudes_[i] += dot_amplitude * dt;
        }

        // Atualiza fases (loop manual)
        for (int i = 0; i < 4; ++i) {
            double omega_i = (std::sin(phases_[i]) > 0) ? omega_swing : omega_stance;
            double acoplamento = 0.0;
            for (int j = 0; j < 4; ++j) {
                if (i == j) continue;
                double w_ij = forca_acoplamento;
                acoplamento += amplitudes_[j] * w_ij * std::sin((phases_[j] - phases_[i]) - target_trot_matrix_[i][j]);
            }
            double dphi = omega_i + acoplamento;
            phases_[i] += dphi * dt;
        }
    }

    std::vector<std::array<double, 3>> calculate_foot_positions()
    {
        std::vector<std::array<double, 3>> positions(4);
        double g_c = this->get_parameter("altura_passo").as_double();
        double g_p = this->get_parameter("profundidade_stance").as_double();
        double passo_base = this->get_parameter("passo_base").as_double();
        double passo_ganho = this->get_parameter("passo_ganho").as_double();
        double passo_max = this->get_parameter("passo_max").as_double();

        double velocidade_linear_total = std::sqrt(v_x_ * v_x_ + v_y_ * v_y_);
        std::array<double, 2> direcao_passo = {1.0, 0.0};
        if (velocidade_linear_total > 1e-6) {
            direcao_passo[0] = v_x_ / velocidade_linear_total;
            direcao_passo[1] = v_y_ / velocidade_linear_total;
        }

        double comprimento_passo_base = std::clamp(passo_base + passo_ganho * velocidade_linear_total, 0.0, passo_max);
        double delta_d_step = omega_z_ * (LARGURA_CORPO / 2.0);

        // Mapeamento: 0:FL, 1:RL, 2:FR, 3:RR
        double comprimentos_passo[] = {
            comprimento_passo_base - delta_d_step, // FL
            comprimento_passo_base - delta_d_step, // RL
            comprimento_passo_base + delta_d_step, // FR
            comprimento_passo_base + delta_d_step  // RR
        };

        for(int i=0; i<4; ++i)
        {
            double r_i = amplitudes_[i];
            double theta_i = phases_[i];
            
            double magnitude_horizontal = -comprimentos_passo[i] * r_i * std::cos(theta_i);
            
            positions[i][0] = magnitude_horizontal * direcao_passo[0]; // X
            positions[i][1] = magnitude_horizontal * direcao_passo[1]; // Y
            // Z (lógica corrigida sem -h)
            positions[i][2] = (std::sin(theta_i) > 0) ? g_c * r_i * std::sin(theta_i) : -g_p * r_i * std::abs(std::sin(theta_i));
        }
        return positions;
    }

    void publish_ik_command(const std::vector<std::array<double, 3>>& positions)
    {
        auto msg = std::make_unique<BodyLegIKTrajectory>();
        caramel_kinematics::msg::BodyLegIK ik_point;
        ik_point.leg_points.reference_link = 1;

        // Mapeamento: 0:FL, 1:RL, 2:FR, 3:RR
        ik_point.leg_points.front_left_leg.x = positions[0][0];
        ik_point.leg_points.front_left_leg.y = positions[0][1];
        ik_point.leg_points.front_left_leg.z = positions[0][2];
        
        ik_point.leg_points.back_left_leg.x = positions[1][0];
        ik_point.leg_points.back_left_leg.y = positions[1][1];
        ik_point.leg_points.back_left_leg.z = positions[1][2];

        ik_point.leg_points.front_right_leg.x = positions[2][0];
        ik_point.leg_points.front_right_leg.y = positions[2][1];
        ik_point.leg_points.front_right_leg.z = positions[2][2];
        
        ik_point.leg_points.back_right_leg.x = positions[3][0];
        ik_point.leg_points.back_right_leg.y = positions[3][1];
        ik_point.leg_points.back_right_leg.z = positions[3][2];

        msg->body_leg_ik_trajectory.push_back(ik_point);
        builtin_interfaces::msg::Duration time;
        time.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period_).count();
        msg->time_from_start.push_back(time);
        
        pub_->publish(std::move(msg));
    }

    // Membros da classe
    rclcpp::Subscription<Twist>::SharedPtr sub_;
    rclcpp::Publisher<BodyLegIKTrajectory>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds timer_period_;
    std::vector<double> phases_;
    std::vector<double> amplitudes_;
    std::vector<std::vector<double>> target_trot_matrix_;
    double v_x_ = 0.0, v_y_ = 0.0, omega_z_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CpgNode>());
    rclcpp::shutdown();
    return 0;
}