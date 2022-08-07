#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "black_mouth_kinematics/InverseKinematics.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

InverseKinematics::InverseKinematics() : Node("inverse_kinematics_node")
{
}

InverseKinematics::~InverseKinematics()
{
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematics>());
  rclcpp::shutdown();
  return 0;
}
