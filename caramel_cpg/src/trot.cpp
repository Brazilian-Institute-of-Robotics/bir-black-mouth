// src/trot.cpp
#include "rclcpp/rclcpp.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include <chrono>
#include <cmath>
#include <vector>
#include <string>

using namespace std::chrono_literals;
using IK_MSG = caramel_kinematics::msg::BodyLegIKTrajectory;

class HopfCPGNode : public rclcpp::Node
{
public:
  HopfCPGNode()
  : Node("hopf_cpg_node")
  {
    // Parâmetros de movimento / CPG
    this->declare_parameter<double>("step_amplitude_x", 0.04);   // deslocamento x (m)
    this->declare_parameter<double>("lift_amplitude_z", 0.03);   // amplitude de elevação (m)
    this->declare_parameter<double>("body_height", 0.0);         // offset em z (base)
    this->declare_parameter<double>("omega", 2.0 * M_PI * 1.0);  // frequência natural (rad/s) — padrão 1 Hz
    this->declare_parameter<double>("mu", 10.0);                 // parâmetro de atração do Hopf (rápido estabilizador)
    this->declare_parameter<double>("r_desired", 1.0);           // amplitude desejada do oscilador (escala interna)
    this->declare_parameter<double>("coupling_K", 6.0);          // ganho de acoplamento genérico
    this->declare_parameter<double>("dt", 0.02);                 // passo de integração (s) = timer period

    // fallback: individual coupling entries (4x4) as flattened list (optional)
    std::vector<double> default_k_flat(16, 0.0);
    this->declare_parameter<std::vector<double>>("coupling_matrix", default_k_flat);

    // Gait desired phase offsets between legs (radians)
    // Ordem de pernas: 0=FR, 1=FL, 2=BR, 3=BL
    // Para trote: FR & BL in-phase (0), FL & BR in-phase (0), e diagonais com fase oposta (pi)
    std::vector<double> default_phase_desired = {0.0, M_PI, M_PI, 0.0};
    this->declare_parameter<std::vector<double>>("phase_desired", default_phase_desired);

    // Publicador e timer
    pub_ = this->create_publisher<IK_MSG>("/cmd_ik", 10);

    double dt = this->get_parameter("dt").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&HopfCPGNode::timer_callback, this)
    );

    // Inicializações internas do CPG
    num_osc_ = 4;
    r_.assign(num_osc_, this->get_parameter("r_desired").as_double()*0.9); // começa quase no desejado
    phi_.assign(num_osc_, 0.0);

    // set initial phases to match desired gait roughly
    auto phase_desired = this->get_parameter("phase_desired").as_double_array();
    if ((int)phase_desired.size() == num_osc_) {
      for (int i = 0; i < num_osc_; ++i) phi_[i] = phase_desired[i];
    } else {
      // default fallback
      phi_[0] = 0.0; phi_[1] = M_PI; phi_[2] = M_PI; phi_[3] = 0.0;
    }

    // coupling matrix
    auto k_flat = this->get_parameter("coupling_matrix").as_double_array();
    coupling_.assign(num_osc_ * num_osc_, 0.0);
    if ((int)k_flat.size() == num_osc_ * num_osc_) {
      for (int i = 0; i < num_osc_ * num_osc_; ++i) coupling_[i] = k_flat[i];
    } else {
      // if not provided, create a simple symmetric matrix that enforces trot:
      // strong coupling between diagonal pairs and weaker between others
      double K = this->get_parameter("coupling_K").as_double();
      for (int i = 0; i < num_osc_; ++i)
        for (int j = 0; j < num_osc_; ++j)
          coupling_[i * num_osc_ + j] = (i == j) ? 0.0 : K * 0.5; // baseline
      // strengthen diagonal pair couplings (FR-BL) and (FL-BR)
      // indices: FR=0, FL=1, BR=2, BL=3
      coupling_[0 * num_osc_ + 3] = K; coupling_[3 * num_osc_ + 0] = K;
      coupling_[1 * num_osc_ + 2] = K; coupling_[2 * num_osc_ + 1] = K;
    }

    RCLCPP_INFO(this->get_logger(), "Hopf CPG node iniciado. dt=%.3f s", dt);
  }

private:
  void timer_callback()
  {
    // Read parameters that can change at runtime
    double step_x = this->get_parameter("step_amplitude_x").as_double();
    double lift_z = this->get_parameter("lift_amplitude_z").as_double();
    double body_h = this->get_parameter("body_height").as_double();
    double omega_base = this->get_parameter("omega").as_double();
    double mu = this->get_parameter("mu").as_double();
    double r_des = this->get_parameter("r_desired").as_double();
    double dt = this->get_parameter("dt").as_double();

    // desired phase offsets (gait)
    auto phase_desired = this->get_parameter("phase_desired").as_double_array();
    std::vector<double> phi_des(num_osc_, 0.0);
    if ((int)phase_desired.size() == num_osc_) {
      for (int i = 0; i < num_osc_; ++i) phi_des[i] = phase_desired[i];
    } else {
      phi_des = {0.0, M_PI, M_PI, 0.0};
    }

    // --- Integrate Hopf oscillators (polar form) ---
    // dr_i/dt = mu * (r_des - r_i) * r_i
    // dphi_i/dt = omega_base + sum_j K_ij * sin(phi_j - phi_i - (phi_des_j - phi_des_i))
    std::vector<double> dr(num_osc_, 0.0);
    std::vector<double> dphi(num_osc_, 0.0);

    for (int i = 0; i < num_osc_; ++i) {
      dr[i] = mu * (r_des - r_[i]) * r_[i];
      double coupling_sum = 0.0;
      for (int j = 0; j < num_osc_; ++j) {
        if (i == j) continue;
        double Kij = coupling_[i * num_osc_ + j];
        double phase_diff_des = phi_des[j] - phi_des[i]; // desired relative phase
        double argument = phi_[j] - phi_[i] - phase_diff_des;
        coupling_sum += Kij * std::sin(argument);
      }
      dphi[i] = omega_base + coupling_sum;
    }

    // Euler integration
    for (int i = 0; i < num_osc_; ++i) {
      r_[i] += dr[i] * dt;
      if (r_[i] < 1e-6) r_[i] = 1e-6; // numeric safety
      phi_[i] += dphi[i] * dt;

      // keep phi in -pi..pi (not strictly necessary)
      if (phi_[i] > M_PI) phi_[i] = std::fmod(phi_[i] + M_PI, 2.0*M_PI) - M_PI;
      if (phi_[i] < -M_PI) phi_[i] = std::fmod(phi_[i] - M_PI, 2.0*M_PI) + M_PI;
    }

    // --- Map oscillator states to leg target positions (x,z) ---
    // Strategy:
    //  - forward/back x: cos(phi) scaled by step_x and by r_i (so amplitude can be modulated)
    //  - lift z: positive only when sin(phi)>0 (stance/swing separation), scaled by lift_z and r_i
    // Leg order: 0=FR,1=FL,2=BR,3=BL
    std::vector<double> leg_x(num_osc_, 0.0);
    std::vector<double> leg_z(num_osc_, 0.0);

    for (int i = 0; i < num_osc_; ++i) {
      double ph = phi_[i];
      double rscale = r_[i]; // allow amplitude modulation via r
      leg_x[i] = step_x * rscale * std::cos(ph);
      double swing = std::sin(ph);
      double lift = (swing > 0.0) ? swing : 0.0;
      leg_z[i] = - (body_h + lift_z * rscale * lift); // negative z = down in many frames; adjust as needed
    }

    // Publish single IK point (you can publish trajectory points if desired)
    auto msg = std::make_unique<IK_MSG>();
    caramel_kinematics::msg::BodyLegIK ik_point;
    ik_point.leg_points.reference_link = 1;

    // Map to message fields
    // front_right = 0, front_left = 1, back_right = 2, back_left = 3
    ik_point.leg_points.front_right_leg.x = leg_x[0];
    ik_point.leg_points.front_right_leg.y = 0.0;
    ik_point.leg_points.front_right_leg.z = leg_z[0];

    ik_point.leg_points.front_left_leg.x = leg_x[1];
    ik_point.leg_points.front_left_leg.y = 0.0;
    ik_point.leg_points.front_left_leg.z = leg_z[1];

    ik_point.leg_points.back_right_leg.x = leg_x[2];
    ik_point.leg_points.back_right_leg.y = 0.0;
    ik_point.leg_points.back_right_leg.z = leg_z[2];

    ik_point.leg_points.back_left_leg.x = leg_x[3];
    ik_point.leg_points.back_left_leg.y = 0.0;
    ik_point.leg_points.back_left_leg.z = leg_z[3];

    msg->body_leg_ik_trajectory.push_back(ik_point);
    builtin_interfaces::msg::Duration time_msg;
    time_msg.sec = 0;
    time_msg.nanosec = 0;
    msg->time_from_start.push_back(time_msg);

    pub_->publish(std::move(msg));
  }

  rclcpp::Publisher<IK_MSG>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int num_osc_;
  std::vector<double> r_;
  std::vector<double> phi_;
  std::vector<double> coupling_; // flattened NxN
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HopfCPGNode>());
  rclcpp::shutdown();
  return 0;
}
