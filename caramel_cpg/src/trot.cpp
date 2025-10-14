#include "rclcpp/rclcpp.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;
using IK_MSG = caramel_kinematics::msg::BodyLegIKTrajectory;
using namespace Eigen;

class HopfCPGNode : public rclcpp::Node
{
public:
  HopfCPGNode()
  : Node("hopf_cpg_node")
  {
    // ==== Parâmetros ====
    this->declare_parameter<double>("step_amplitude_x", 0.04);
    this->declare_parameter<double>("stance_lift_z", 0.005);
    this->declare_parameter<double>("swing_lift_z", 0.05);
    this->declare_parameter<double>("body_height", 0.0);
    this->declare_parameter<double>("mu", 10.0);
    this->declare_parameter<double>("r_desired", 1.0);
    this->declare_parameter<double>("coupling_K", 20.0);
    this->declare_parameter<double>("dt", 0.02);
    this->declare_parameter<double>("omega_stance", 2.0 * M_PI * 0.8);  // 0.8 Hz
    this->declare_parameter<double>("omega_swing", 2.0 * M_PI * 1.2);   // 1.2 Hz

    // === Fase desejada (trote) ===
    std::vector<double> default_phase_des = {0.0, M_PI, M_PI, 0.0};
    this->declare_parameter<std::vector<double>>("phase_desired", default_phase_des);

    // === Offset de fase (ajuste estático de postura) ===
    std::vector<double> default_phase_offset = {0.1, -0.1, 0.1, -0.1}; 
    this->declare_parameter<std::vector<double>>("phase_offset", default_phase_offset);

    // === Publicador e Timer ===
    pub_ = this->create_publisher<IK_MSG>("/cmd_ik", 10);
    double dt = this->get_parameter("dt").as_double();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&HopfCPGNode::timer_callback, this)
    );

    // ==== Inicialização ====
    num_osc_ = 4;
    r_ = VectorXd::Constant(num_osc_, this->get_parameter("r_desired").as_double() * 0.9);
    phi_ = VectorXd::Zero(num_osc_);

    auto phase_des = this->get_parameter("phase_desired").as_double_array();
    for (int i = 0; i < num_osc_ && i < (int)phase_des.size(); ++i)
      phi_[i] = phase_des[i];

    // ==== Matriz de acoplamento ====
    double K = this->get_parameter("coupling_K").as_double();
    coupling_ = MatrixXd::Constant(num_osc_, num_osc_, K * 0.5);
    for (int i = 0; i < num_osc_; ++i) coupling_(i, i) = 0.0;
    // diagonais fortes
    coupling_(0, 3) = coupling_(3, 0) = K;
    coupling_(1, 2) = coupling_(2, 1) = K;

    RCLCPP_INFO(this->get_logger(), "Hopf CPG node iniciado com dt=%.3f s", dt);
  }

private:
  void timer_callback()
  {
    // ==== Leitura dos parâmetros ====
    double step_x = this->get_parameter("step_amplitude_x").as_double();
    double lift_stance = this->get_parameter("stance_lift_z").as_double();
    double lift_swing = this->get_parameter("swing_lift_z").as_double();
    double body_h = this->get_parameter("body_height").as_double();
    double mu = this->get_parameter("mu").as_double();
    double r_des = this->get_parameter("r_desired").as_double();
    double dt = this->get_parameter("dt").as_double();
    double omega_stance = this->get_parameter("omega_stance").as_double();
    double omega_swing = this->get_parameter("omega_swing").as_double();

    auto phase_desired = this->get_parameter("phase_desired").as_double_array();
    VectorXd phi_des(num_osc_);
    for (int i = 0; i < num_osc_ && i < (int)phase_desired.size(); ++i)
      phi_des[i] = phase_desired[i];

    // ==== Leitura dos offsets de fase ====
    auto phase_offset_vec = this->get_parameter("phase_offset").as_double_array();
    VectorXd phase_offset(num_osc_);
    for (int i = 0; i < num_osc_ && i < (int)phase_offset_vec.size(); ++i)
      phase_offset[i] = phase_offset_vec[i];

    // ==== Integração do CPG ====
    VectorXd dr(num_osc_);
    VectorXd dphi(num_osc_);

    for (int i = 0; i < num_osc_; ++i)
    {
      dr[i] = mu * (r_des - r_[i]) * r_[i];

      double omega_i = (std::sin(phi_[i]) > 0.0) ? omega_swing : omega_stance;

      double coupling_sum = 0.0;
      for (int j = 0; j < num_osc_; ++j)
      {
        if (i == j) continue;
        double phase_diff_des = phi_des[j] - phi_des[i];
        double arg = phi_[j] - phi_[i] - phase_diff_des;
        coupling_sum += coupling_(i, j) * std::sin(arg);
      }
      dphi[i] = omega_i + coupling_sum;
    }

    // === Integração numérica (Euler) ===
    r_ += dr * dt;
    phi_ += dphi * dt;

    for (int i = 0; i < num_osc_; ++i)
      phi_[i] = std::fmod(phi_[i] + M_PI, 2.0 * M_PI) - M_PI;

    // ==== Mapeamento para coordenadas de perna ====
    VectorXd leg_x(num_osc_);
    VectorXd leg_z(num_osc_);

    for (int i = 0; i < num_osc_; ++i)
    {
      double ph = phi_[i] + phase_offset[i]; // <--- offset aplicado aqui
      double rscale = r_[i];
      bool swing_phase = (std::sin(ph) > 0.0);

      leg_x[i] = -step_x * rscale * std::cos(ph);
      double lift = swing_phase ? lift_swing : lift_stance;
      leg_z[i] = lift * rscale * std::sin(ph) + body_h;
    }

    // ==== Publicação ====
    auto msg = std::make_unique<IK_MSG>();
    caramel_kinematics::msg::BodyLegIK ik_point;
    ik_point.leg_points.reference_link = 1;

    ik_point.leg_points.front_right_leg.x = leg_x[0];
    ik_point.leg_points.front_right_leg.z = leg_z[0];

    ik_point.leg_points.front_left_leg.x = leg_x[1];
    ik_point.leg_points.front_left_leg.z = leg_z[1];

    ik_point.leg_points.back_right_leg.x = leg_x[2];
    ik_point.leg_points.back_right_leg.z = leg_z[2];

    ik_point.leg_points.back_left_leg.x = leg_x[3];
    ik_point.leg_points.back_left_leg.z = leg_z[3];

    msg->body_leg_ik_trajectory.push_back(ik_point);

    builtin_interfaces::msg::Duration t;
    t.sec = 0; t.nanosec = 0;
    msg->time_from_start.push_back(t);

    pub_->publish(std::move(msg));
  }

  // === Membros ===
  rclcpp::Publisher<IK_MSG>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int num_osc_;
  VectorXd r_;
  VectorXd phi_;
  MatrixXd coupling_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HopfCPGNode>());
  rclcpp::shutdown();
  return 0;
}
