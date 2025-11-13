#include "caramel_cpg/hopf_cpg_node.hpp"
#include <Eigen/Dense>
#include <cmath> // Necessário para std::fabs e std::sqrt

using namespace Eigen;

HopfCPGNode::HopfCPGNode()
: Node("hopf_cpg_node")
{
  this->declare_parameter<double>("stance_lift_z", 0.0035);
  this->declare_parameter<double>("swing_lift_z", 0.045);
  this->declare_parameter<double>("body_height", 0.0);
  this->declare_parameter<double>("mu", 10.0);
  this->declare_parameter<double>("r_desired", 1.0);
  this->declare_parameter<double>("coupling_K", 1.0);
  this->declare_parameter<double>("omega_stance", 0.5);
  this->declare_parameter<double>("omega_swing", 2.5);
  this->declare_parameter<double>("body_offset_x", 0.03);
  this->declare_parameter<double>("body_offset_y", 0.0);
  this->declare_parameter<double>("body_offset_z", 0.0);
  
  this->declare_parameter<double>("turn_scale", 0.25);
  this->declare_parameter<double>("freq_scale", 1.5);
  this->declare_parameter<double>("max_step_x", 0.05);

  std::vector<double> default_phase_des = {0.0, M_PI, M_PI, 0.0};
  this->declare_parameter<std::vector<double>>("phase_desired", default_phase_des);

  sub_vel_ = this->create_subscription<TWIST_MSG>(
    "/cmd_vel", // Lembre-se que o launch remapeia isso para /cmd_vel_cpg
    10,
    std::bind(&HopfCPGNode::cmd_vel_callback, this, std::placeholders::_1)
  );

  pub_ = this->create_publisher<IK_MSG>("/cmd_ik", 10);
  
  // MUDANÇA: Timer ajustado de 5ms (200Hz) para 33ms (~30Hz)
  // Isso reduz a carga no nó de IK e evita timeouts.
  timer_ = this->create_wall_timer(
    33ms, 
    std::bind(&HopfCPGNode::timer_callback, this)
  );

  last_cmd_vel_.linear.x = 0.0;
  last_cmd_vel_.linear.y = 0.0;
  last_cmd_vel_.angular.z = 0.0;

  num_osc_ = 4;
  r_ = VectorXd::Constant(num_osc_, this->get_parameter("r_desired").as_double() * 0.9);
  phi_ = VectorXd::Zero(num_osc_);

  auto phase_des = this->get_parameter("phase_desired").as_double_array();
  for (int i = 0; i < num_osc_ && i < (int)phase_des.size(); ++i)
    phi_[i] = phase_des[i];

  double K = this->get_parameter("coupling_K").as_double();
  coupling_ = MatrixXd::Constant(num_osc_, num_osc_, K * 0.5);
  for (int i = 0; i < num_osc_; ++i) coupling_(i, i) = 0.0;
  coupling_(0, 3) = coupling_(3, 0) = K;
  coupling_(1, 2) = coupling_(2, 1) = K;

  omega_stance_base_ = 2.0 * M_PI * this->get_parameter("omega_stance").as_double();
  omega_swing_base_ = 2.0 * M_PI * this->get_parameter("omega_swing").as_double();
  turn_scale_ = this->get_parameter("turn_scale").as_double();

  last_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Hopf CPG node iniciado com entrada de velocidade (taxa 30Hz).");
}

void HopfCPGNode::cmd_vel_callback(const TWIST_MSG::SharedPtr msg)
{
  last_cmd_vel_ = *msg;
}

void HopfCPGNode::timer_callback()
{
  auto current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;
  // Clamp de DT ajustado para o novo timer de 33ms
  if (dt <= 0.0 || dt > 0.1) dt = 0.033; 

  // === 1. Obter Comandos de Velocidade ===
  double vx = last_cmd_vel_.linear.x;
  double vy = last_cmd_vel_.linear.y;
  double vth = last_cmd_vel_.angular.z;

  // --- MUDANÇA: Adiciona o Deadzone ---
  // Se os comandos forem muito pequenos (drift do joystick), zere-os.
  const double linear_deadzone = 0.02;  // 2 cm/s
  const double angular_deadzone = 0.05; // ~3 graus/s

  if (std::sqrt(vx*vx + vy*vy) < linear_deadzone)
  {
    vx = 0.0;
    vy = 0.0;
  }
  if (std::fabs(vth) < angular_deadzone)
  {
    vth = 0.0;
  }
  // --- Fim da Mudança ---

  // === 2. Obter Parâmetros ===
  double lift_stance = this->get_parameter("stance_lift_z").as_double();
  double lift_swing = this->get_parameter("swing_lift_z").as_double();
  double body_h = this->get_parameter("body_height").as_double();
  double mu = this->get_parameter("mu").as_double();
  double r_des = this->get_parameter("r_desired").as_double();
  double freq_scale = this->get_parameter("freq_scale").as_double();
  
  double f_base_hz = this->get_parameter("omega_stance").as_double();

  double body_offset_x = this->get_parameter("body_offset_x").as_double();
  double body_offset_y = this->get_parameter("body_offset_y").as_double();
  double body_offset_z = this->get_parameter("body_offset_z").as_double();

  // === 3. Mapear Velocidade para Parâmetros do CPG ===

  // --- A. Frequência ---
  double vel_magnitude = std::sqrt(vx*vx + vy*vy + (vth*turn_scale_)*(vth*turn_scale_));
  double omega_scale = 1.0 + (vel_magnitude * freq_scale);
  double omega_stance = omega_stance_base_ * omega_scale;
  double omega_swing = omega_swing_base_ * omega_scale;
  
  double current_f_hz = f_base_hz * omega_scale;

  // --- B. Amplitude (CORRIGIDO) ---
  double target_amplitude_x = 0.0;
  double target_amplitude_y = 0.0;

  if (current_f_hz > 0.01) 
  {
      target_amplitude_x = vx / (4.0 * current_f_hz);
      target_amplitude_y = vy / (4.0 * current_f_hz);
  }
  
  VectorXd step_x_amp(num_osc_);
  VectorXd step_y_amp(num_osc_);
  
  step_x_amp[0] = target_amplitude_x - vth * turn_scale_;
  step_x_amp[1] = target_amplitude_x + vth * turn_scale_;
  step_x_amp[2] = target_amplitude_x - vth * turn_scale_;
  step_x_amp[3] = target_amplitude_x + vth * turn_scale_;

  step_y_amp.setConstant(target_amplitude_y); 

  // --- 4. Lógica de Integração do CPG ---
  auto phase_desired = this->get_parameter("phase_desired").as_double_array();
  VectorXd phi_des(num_osc_);
  for (int i = 0; i < num_osc_ && i < (int)phase_desired.size(); ++i)
    phi_des[i] = phase_desired[i];

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

  r_ += dr * dt;
  phi_ += dphi * dt;
  for (int i = 0; i < num_osc_; ++i)
    phi_[i] = std::fmod(phi_[i] + M_PI, 2.0 * M_PI) - M_PI;

  // --- 5. Mapeamento Cartesiano ---
  VectorXd leg_x(num_osc_);
  VectorXd leg_y(num_osc_);
  VectorXd leg_z(num_osc_);

  for (int i = 0; i < num_osc_; ++i)
  {
    double ph = phi_[i];
    double rscale = r_[i];
    bool swing_phase = (std::sin(ph) > 0.0);
    
    leg_x[i] = -step_x_amp[i] * rscale * std::cos(ph);
    leg_y[i] = -step_y_amp[i] * rscale * std::cos(ph);
    
    double lift = swing_phase ? lift_swing : lift_stance;
    leg_z[i] = lift * rscale * std::sin(ph) + body_h;
  }

  // --- 6. Publicação da Mensagem ---
  auto msg = std::make_unique<IK_MSG>();
  caramel_kinematics::msg::BodyLegIK ik_point;
  ik_point.leg_points.reference_link = 1;

  ik_point.leg_points.front_right_leg.x = leg_x[0];
  ik_point.leg_points.front_right_leg.y = leg_y[0];
  ik_point.leg_points.front_right_leg.z = leg_z[0];

  ik_point.leg_points.front_left_leg.x = leg_x[1];
  ik_point.leg_points.front_left_leg.y = leg_y[1];
  ik_point.leg_points.front_left_leg.z = leg_z[1];

  ik_point.leg_points.back_right_leg.x = leg_x[2];
  ik_point.leg_points.back_right_leg.y = leg_y[2];
  ik_point.leg_points.back_right_leg.z = leg_z[2];

  ik_point.leg_points.back_left_leg.x = leg_x[3];
  ik_point.leg_points.back_left_leg.y = leg_y[3];
  ik_point.leg_points.back_left_leg.z = leg_z[3];

  ik_point.body_position.x = body_offset_x;
  ik_point.body_position.y = body_offset_y;
  ik_point.body_position.z = body_offset_z;

  msg->body_leg_ik_trajectory.push_back(ik_point);
  builtin_interfaces::msg::Duration t;
  t.sec = 0; t.nanosec = 0;
  msg->time_from_start.push_back(t);
  pub_->publish(std::move(msg));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HopfCPGNode>());
  rclcpp::shutdown();
  return 0;
}