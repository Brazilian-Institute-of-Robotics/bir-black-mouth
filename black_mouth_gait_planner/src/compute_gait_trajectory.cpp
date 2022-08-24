#include "black_mouth_gait_planner/srv/compute_gait_trajectory.hpp"

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <chrono>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/vector3.hpp"

void computeGaitTrajectory(
    const std::shared_ptr<
        black_mouth_gait_planner::srv::ComputeGaitTrajectory::Request>
        request,
    std::shared_ptr<
        black_mouth_gait_planner::srv::ComputeGaitTrajectory::Response>
        response) {
    auto start = std::chrono::steady_clock::now();
    auto time =
        Eigen::ArrayXd::LinSpaced(request->resolution, 0.0, request->period)
            .transpose();
    auto k = 2 * M_PI / request->period * time;
    auto xy_common = (k - k.sin()) / (2 * M_PI);
    auto x_axis =
        (request->landing_point.x - request->initial_point.x) * xy_common +
        request->initial_point.x;
    auto y_axis =
        (request->landing_point.y - request->initial_point.y) * xy_common +
        request->initial_point.y;
    auto z_axis =
        request->height * (1 - k.cos()) / 2 + request->initial_point.z;

    std::vector<geometry_msgs::msg::Vector3> points;
    points.resize(request->resolution);
    std::vector<builtin_interfaces::msg::Duration> time_from_start;
    time_from_start.resize(request->resolution);

    for (int i = 0; i < request->resolution; i++) {
        geometry_msgs::msg::Vector3 point;
        point.x = x_axis[i];
        point.y = y_axis[i];
        point.z = z_axis[i];
        points[i] = point;
        time_from_start[i] = rclcpp::Duration::from_seconds(time[i]);
    }

    response->points = points;
    response->time_from_start = time_from_start;

    auto end = std::chrono::steady_clock::now();
    std::cout << "Elapsed time in microseconds: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        << " µs" << std::endl;

    // std::cout << "X: " << x_axis << std::endl;
    // std::cout << "Y: " << y_axis << std::endl;
    // std::cout << "Z: " << z_axis << std::endl;
    // std::cout << "T: " << time << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("compute_gait_trajectory_server");

    rclcpp::Service<black_mouth_gait_planner::srv::ComputeGaitTrajectory>::
        SharedPtr service = node->create_service<
            black_mouth_gait_planner::srv::ComputeGaitTrajectory>(
            "compute_gait_trajectory", &computeGaitTrajectory);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Ready to compute gait trajectory.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}