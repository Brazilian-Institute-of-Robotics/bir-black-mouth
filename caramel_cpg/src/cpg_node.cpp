#include "rclcpp/rclcpp.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
// --- CORREÇÃO AQUI: Trocado '/' por '::' ---
using IK_MSG = caramel_kinematics::msg::BodyLegIKTrajectory;

class FlexaoNode : public rclcpp::Node
{
public:
    FlexaoNode() : Node("cpg_node")
    {
        // Parâmetros com valores corrigidos
        this->declare_parameter("altura_maxima", 0.05);
        this->declare_parameter("altura_minima", -0.03);
        this->declare_parameter("frequencia", 0.1);

        pub_ = this->create_publisher<IK_MSG>("/cmd_ik", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&FlexaoNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nó de Flexão iniciado (apenas em Z, valores corrigidos).");
    }

private:
    void timer_callback()
    {
        double altura_max = this->get_parameter("altura_maxima").as_double();
        double altura_min = this->get_parameter("altura_minima").as_double();
        double freq = this->get_parameter("frequencia").as_double();
        
        double z_amplitude = (altura_min - altura_max) / 2.0;
        double z_centro = (altura_max + altura_min) / 2.0;

        double tempo_atual = this->get_clock()->now().seconds();
        double z_alvo = z_centro + z_amplitude * sin(2.0 * M_PI * freq * tempo_atual);

        auto msg = std::make_unique<IK_MSG>();
        caramel_kinematics::msg::BodyLegIK ik_point;
        ik_point.leg_points.reference_link = 1;

        ik_point.leg_points.front_right_leg.x = 0.0;
        ik_point.leg_points.front_right_leg.y = 0.0;
        ik_point.leg_points.front_right_leg.z = z_alvo;

        ik_point.leg_points.front_left_leg.x = 0.0;
        ik_point.leg_points.front_left_leg.y = 0.0;
        ik_point.leg_points.front_left_leg.z = z_alvo;

        ik_point.leg_points.back_right_leg.x = 0.0;
        ik_point.leg_points.back_right_leg.y = 0.0;
        ik_point.leg_points.back_right_leg.z = z_alvo;

        ik_point.leg_points.back_left_leg.x = 0.0;
        ik_point.leg_points.back_left_leg.y = 0.0;
        ik_point.leg_points.back_left_leg.z = z_alvo;
        
        msg->body_leg_ik_trajectory.push_back(ik_point);
        
        builtin_interfaces::msg::Duration time;
        time.sec = 0;
        time.nanosec = 0; // 50ms de duração
        msg->time_from_start.push_back(time);
        
        pub_->publish(std::move(msg));
    }

    rclcpp::Publisher<IK_MSG>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlexaoNode>());
    rclcpp::shutdown();
    return 0;
}