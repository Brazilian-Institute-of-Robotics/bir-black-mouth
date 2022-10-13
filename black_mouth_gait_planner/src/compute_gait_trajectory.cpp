#include "black_mouth_gait_planner/srv/compute_gait_trajectory.hpp"

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <chrono>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"

void computeGaitTrajectory(
    const std::shared_ptr<
        black_mouth_gait_planner::srv::ComputeGaitTrajectory::Request>
        request,
    std::shared_ptr<
        black_mouth_gait_planner::srv::ComputeGaitTrajectory::Response>
        response) {

    // auto start = std::chrono::steady_clock::now();

    int N_first_fraction = (int)(request->resolution * request->resolution_first_fraction);
    double T_first_fraction = (double)(request->period * request->period_first_fraction);

    Eigen::ArrayXd time(request->resolution); 

    if(request->resolution_first_fraction != 1.0 && request->period_first_fraction != 1.0){
        auto time1 = Eigen::ArrayXd::LinSpaced(N_first_fraction, 0.0, T_first_fraction);
        auto time2 = Eigen::ArrayXd::LinSpaced(request->resolution - N_first_fraction + 1,
            T_first_fraction, 
            request->period);

        for (int i=0; i < request->resolution; i++){
            time[i] = i < N_first_fraction ? time1[i] : time2[i - N_first_fraction + 1];
        }
    }
    else {
        time = Eigen::ArrayXd::LinSpaced(request->resolution, 0.0,
            request->period);
    }

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

    response->points.resize(request->resolution);
    response->time_from_start.resize(request->resolution);

    for (int i = 0; i < request->resolution; i++) {
        response->points[i].x = x_axis[i];
        response->points[i].y = y_axis[i];
        response->points[i].z = z_axis[i];
        response->time_from_start[i] = rclcpp::Duration::from_seconds(time[i]);
    }

    // auto end = std::chrono::steady_clock::now();
    // std::cout << "Elapsed time in microseconds: "
    //     << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
    //     << " µs" << std::endl;
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