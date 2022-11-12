#!/usr/bin/env python3
import rclpy
import time
import sys
import csv
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from caramel_kinematics.msg import BodyLegIKTrajectory, BodyLegIK, AllLegPoints
from caramel_gait_planner.srv import ComputeGaitTrajectory


class FootFollowTrajectory(Node):
    def __init__(self):
        super().__init__('foot_follow_trajectory')

        self.declare_parameter('gait_period', 0.5)
        self.declare_parameter('gait_height', 0.05)
        self.declare_parameter('body_height', 0.0)
        self.declare_parameter('resolution_first_fraction', 1.0)
        self.declare_parameter('period_first_fraction', 1.0)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)

        self.gait_period = self.get_parameter(
            'gait_period').get_parameter_value().double_value
        self.gait_height = self.get_parameter(
            'gait_height').get_parameter_value().double_value
        self.body_height = self.get_parameter(
            'body_height').get_parameter_value().double_value
        self.resolution_first_fraction = self.get_parameter(
            'resolution_first_fraction').get_parameter_value().double_value
        self.period_first_fraction = self.get_parameter(
            'period_first_fraction').get_parameter_value().double_value
        self.x_landing_point = self.get_parameter(
            'x').get_parameter_value().double_value
        self.y_landing_point = self.get_parameter(
            'y').get_parameter_value().double_value

        self.ik_publisher_ = self.create_publisher(BodyLegIKTrajectory,
                                                   '/cmd_ik',
                                                   10)

        self.default_feet_pose_subscriber = self.create_subscription(AllLegPoints,
                                                                     'feet_poses',
                                                                     self.default_feet_points,
                                                                     10)
        self.default_feet_pose_msg = AllLegPoints()

        self.finished = False
        self.start_time = 0.0

        self.max_z = 0.0

        self.gait_x_length = 0
        self.gait_theta_length = 0
        self.gait_y_length = 0

        self.timer_period = 0.02
        self.ik_timer = self.create_timer(
            self.timer_period, self.timerCallback)
        self.ik_timer.cancel()

        self.gait_res = int(self.gait_period/self.timer_period)

        self.point_counter = 0

        self.support_node = Node('foot_follow_trajectory_support')

        self.traj_client = self.support_node.create_client(
            ComputeGaitTrajectory, 'compute_gait_trajectory')
        while not self.traj_client.wait_for_service(1.0):
            if not rclpy.ok():
                self.get_logger().error(
                    f"Interrupted while waiting for the {self.traj_client.srv_name} service. Exiting.")
                return
            self.get_logger().info(
                f"{self.traj_client.srv_name} service not available, waiting...")

        # ---------------------------------------------------------------------------
        self.leg_request = ComputeGaitTrajectory.Request()
        self.leg_request.initial_point = Point()
        self.leg_request.landing_point = Point(
            x=self.x_landing_point, y=self.y_landing_point)
        self.leg_request.period = self.gait_period
        self.leg_request.height = self.gait_height
        self.leg_request.resolution = self.gait_res
        self.leg_request.resolution_first_fraction = self.resolution_first_fraction
        self.leg_request.period_first_fraction = self.period_first_fraction
        self.leg_response = None

        self.point_counter = 0

        # Set initial position
        self.msg = BodyLegIKTrajectory()
        self.msg.body_leg_ik_trajectory.append(BodyLegIK())
        self.msg.body_leg_ik_trajectory[0].leg_points.reference_link = 1
        self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = self.leg_request.initial_point
        self.msg.time_from_start.append(Duration())

        self.get_logger().info("Ready", once=True)

    def default_feet_points(self, msg):
        self.default_feet_pose_msg = msg
        self.max_z = msg.front_left_leg.z if msg.front_left_leg.z > self.max_z else self.max_z

    def timerCallback(self):

        self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = self.leg_response.points[self.point_counter]

        # Publish
        if self.start_time == 0.0:
            self.start_time = time.time()

        self.ik_publisher_.publish(self.msg)

        # Update time and state counter
        if self.point_counter == self.gait_res - 1:
            self.finished = True
            self.ik_timer.cancel()
            print('fn', self.finished)
        else:
            self.point_counter += 1


def main(args=None):
    rclpy.init(args=args)

    follow_trajectory_node = FootFollowTrajectory()

    # Get first body trajectory
    future = follow_trajectory_node.traj_client.call_async(
        follow_trajectory_node.leg_request)
    rclpy.spin_until_future_complete(
        follow_trajectory_node.support_node, future)
    follow_trajectory_node.leg_response = future.result()

    follow_trajectory_node.ik_timer.reset()

    while (not follow_trajectory_node.finished or 
           follow_trajectory_node.default_feet_pose_msg.front_left_leg.z > 0.001):
        rclpy.spin_once(follow_trajectory_node)
        
    follow_trajectory_node.elapsed_time = time.time() - follow_trajectory_node.start_time

    print("FINAL:\n")
    print("\tX = "+str(follow_trajectory_node.default_feet_pose_msg.front_left_leg.x))
    print("\tY = "+str(follow_trajectory_node.default_feet_pose_msg.front_left_leg.y))
    print("\tZ = "+str(follow_trajectory_node.max_z))
    print("\tTime = "+str(follow_trajectory_node.elapsed_time))
    
    with open('/home/ubuntu/bm_ws/src/bir-black-mouth/caramel_data_analysis/csv/feet_trajectory.csv', 
              'a', newline='') as csv_file:
        
        tipo_passo = "x: " + str(follow_trajectory_node.x_landing_point) + \
                    " y: " + str(follow_trajectory_node.y_landing_point)
                 
        fieldnames = ['tipo_passo', 'tempo', 'x_final', 'y_final', 'z_max']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)    
        writer.writerow({'tipo_passo': tipo_passo, 
                         'tempo': follow_trajectory_node.elapsed_time, 
                         'x_final': follow_trajectory_node.default_feet_pose_msg.front_left_leg.x, 
                         'y_final': follow_trajectory_node.default_feet_pose_msg.front_left_leg.y, 
                         'z_max': follow_trajectory_node.max_z})
        

    follow_trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
