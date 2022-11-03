#include "caramel_gait_planner/srv/compute_gait_trajectory.hpp"
#include "caramel_kinematics/msg/body_leg_ik_trajectory.hpp"
#include "caramel_kinematics/srv/inv_kinematics.hpp"

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

int computeGaitTrajectory(
    rclcpp::Client<caramel_gait_planner::srv::ComputeGaitTrajectory>::SharedPtr  client_gait, rclcpp::Node::SharedPtr node)
    {
        auto request_gait = std::make_shared<caramel_gait_planner::srv::ComputeGaitTrajectory::Request>();

        request_gait->landing_point.x= 0.05;
        request_gait->landing_point.y= 0.0;
        request_gait->landing_point.z= 0.0;
        request_gait->initial_point.x= 0.0;
        request_gait->initial_point.y= 0.0;
        request_gait->initial_point.z= 0.0;
        request_gait->resolution= 10;
        request_gait->period= 10.0;
        request_gait->height= 0.05;

        while (!client_gait->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Computing gait trajectory");
                return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto result = client_gait->async_send_request(request_gait);


        if (rclcpp::spin_until_future_complete(node, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {

            rclcpp::Publisher<caramel_kinematics::msg::BodyLegIKTrajectory>::SharedPtr publisher_ik =   node->create_publisher<caramel_kinematics::msg::BodyLegIKTrajectory>("cmd_ik", 10);

            auto msg = caramel_kinematics::msg::BodyLegIKTrajectory();

            msg.body_leg_ik_trajectory.resize(request_gait->resolution);
            msg.time_from_start.resize(request_gait->resolution);

            auto aux_result = result.get();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending ik request");
            for (int i = 0; i < request_gait->resolution; i++) {

                msg.time_from_start[i].sec = aux_result->time_from_start[i].sec;
                msg.time_from_start[i].nanosec =  aux_result->time_from_start[i].nanosec;;

                msg.body_leg_ik_trajectory[i].leg_points.reference_link = caramel_kinematics::msg::AllLegPoints::FOOT_LINK_AS_REFERENCE;

                msg.body_leg_ik_trajectory[i].leg_points.front_right_leg.x= aux_result->points[i].x;
                msg.body_leg_ik_trajectory[i].leg_points.front_right_leg.y= aux_result->points[i].y;
                msg.body_leg_ik_trajectory[i].leg_points.front_right_leg.z= aux_result->points[i].z;

                publisher_ik->publish(msg);
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Send point %i", i);
            }

        } else {

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        }

        return 0;
    }



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("compute_ik_gait_trajectory_server");

    rclcpp::Client<caramel_gait_planner::srv::ComputeGaitTrajectory>::SharedPtr client_gait =
        node->create_client<caramel_gait_planner::srv::ComputeGaitTrajectory>("compute_gait_trajectory");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Ready to compute gait trajectory.");

    computeGaitTrajectory(client_gait, node);

    rclcpp::spin(node);
    rclcpp::shutdown();
}