#!/usr/bin/env python3
import rclpy
import time
import numpy as np
from rclpy.node import Node
from black_mouth_control.msg import BodyControl
from black_mouth_kinematics.msg import BodyLegIKTrajectory, BodyLegIK
from geometry_msgs.msg import Vector3, Point, Twist
from builtin_interfaces.msg import Duration
from black_mouth_gait_planner.srv import ComputeGaitTrajectory


class TestTrot(Node):
    def __init__(self):
        super().__init__('test_trot_node')

        self.body_rotation = Vector3()

        self.ik_publisher_ = self.create_publisher(
            BodyLegIKTrajectory, '/cmd_ik', 10)
        self.body_control_subscriber = self.create_subscription(BodyControl,
                                                                '/body_control',
                                                                self.bodyCallback,
                                                                10)
        self.body_control_subscriber

        self.cmd_vel_subscriber = self.create_subscription(Twist,
                                                           '/cmd_vel',
                                                           self.cmd_vel_cb,
                                                           10)
        self.gait_x_length = 0
        self.gait_theta_length = 0
        self.gait_y_length = 0
        
        self.cmd_vel_msg = Twist()

        timer_period = 0.02
        self.ik_timer = self.create_timer(timer_period, self.timerCallback)
        self.ik_timer.cancel()

        self.start_time = time.time()
        self.point_counter = 0

        # self.traj_client = self.create_client(
        #     ComputeGaitTrajectory, 'compute_gait_trajectory')
        self.support_node = Node('test_trot_node_support')
        self.traj_client = self.support_node.create_client(
            ComputeGaitTrajectory, 'compute_gait_trajectory')

        self.get_logger().info("Waiting for server")
        while not self.traj_client.wait_for_service(1.0):
            self.get_logger().info("...", once=True)

        self.gait_period = 0.5  # secs
        self.gait_res = int(self.gait_period/timer_period)  # secs
        # self.gait_res = 7  # secs

        self.first_step = True

        self.lower_body = 0.0
        self.lower_leg = -0.0045

        self.req_half_leg = ComputeGaitTrajectory.Request()
        self.req_half_leg.landing_point = Point(x=0.02)
        self.req_half_leg.period = self.gait_period
        self.req_half_leg.height = 0.05
        self.req_half_leg.resolution = self.gait_res
        self.req_half_leg.resolution_first_fraction = 0.33
        self.req_half_leg.period_first_fraction = 0.66

        self.req_leg = ComputeGaitTrajectory.Request()
        self.req_leg.initial_point = Point(z=self.lower_leg)
        self.req_leg.landing_point = Point(x=0.04, z=self.lower_leg)
        self.req_leg.period = self.gait_period
        self.req_leg.height = 0.05
        self.req_leg.resolution = self.gait_res
        self.req_leg.resolution_first_fraction = 0.33
        self.req_leg.period_first_fraction = 0.66

        self.req_minus_leg = ComputeGaitTrajectory.Request()
        self.req_minus_leg.initial_point = Point(x=-0.02)
        self.req_minus_leg.landing_point = Point(x=0.02)
        self.req_minus_leg.period = self.gait_period
        self.req_minus_leg.height = 0.05
        self.req_minus_leg.resolution = self.gait_res
        self.req_minus_leg.resolution_first_fraction = 0.33
        self.req_minus_leg.period_first_fraction = 0.66

        self.req_body1 = ComputeGaitTrajectory.Request()
        self.req_body1.initial_point = Point(x=0.005, z=self.lower_body+0.002)
        self.req_body1.landing_point = Point(x=0.025, z=self.lower_body+0.002)
        self.req_body1.period = self.gait_period
        self.req_body1.height = 0.005
        self.req_body1.resolution = self.gait_res

        self.req_body2 = ComputeGaitTrajectory.Request()
        self.req_body2.initial_point = Point(x=0.025, z=self.lower_body+0.002)
        self.req_body2.landing_point = Point(x=0.045, z=self.lower_body+0.002)
        self.req_body2.period = self.gait_period
        self.req_body2.height = 0.005
        self.req_body2.resolution = self.gait_res

        self.response_half_leg = None
        self.response_minus_leg = None
        self.response_leg = None
        self.response_body1 = None
        self.response_body2 = None

        self.get_logger().info("Setting up states...")
        self.state = 0
        self.start_time = 0.0
        self.point_counter = 0
        self.t1 = Duration(sec=0, nanosec=int(timer_period*1e9))

        self.msg = BodyLegIKTrajectory()
        self.msg.body_leg_ik_trajectory.append(BodyLegIK())
        self.msg.body_leg_ik_trajectory[0].leg_points.reference_link = 1
        self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg.z = self.lower_leg
        self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg.z = self.lower_leg
        self.msg.time_from_start.append(self.t1)

        self.Length = 0.245  # 0.2291
        self.Width = 0.225  # 0.140
        self.origin = 0
        self.last_step_l = 0.0
        self.create_body_matrix()
        
        
    def create_body_matrix(self):
        self.coord_orig = np.array([[self.origin-self.Width/2, self.origin-self.Length/2 - self.last_step_l/2],
                                    [self.origin+self.Width/2, self.origin-self.Length/2],
                                    [self.origin+self.Width/2, self.origin+self.Length/2 - self.last_step_l/2],
                                    [self.origin, self.origin],
                                    [self.origin-self.Width/2, self.origin+self.Length/2]])
        self.coord_orig = np.vstack((self.coord_orig, self.coord_orig[0]))

    def bodyCallback(self, msg):
        self.body_rotation = msg.body_rotation

    def cmd_vel_cb(self, msg):
        self.cmd_vel_msg = msg
        # self.gait_x_length = msg.linear.x*(self.gait_period*4)
        # self.gait_y_length = msg.linear.y*(self.gait_period*4)
        # self.gait_theta_length = msg.angular.z*(self.gait_period*4)

    def update_positions(mat, linear_x, linear_y,  theta):
        result = np.ones((mat.shape[0], 3))
        result[:, :-1] = mat

        rotMat = np.array([[np.cos(-theta), -1*np.sin(-theta), 0],
                           [np.sin(-theta), np.cos(-theta), 0],
                           [0, 0, 1]])

        x = linear_x*np.cos(theta) - linear_y*np.sin(theta)
        y = linear_x*np.sin(theta) + linear_y*np.cos(theta)
        transMat = np.array([[1, 0, y],
                            [0, 1, x],
                            [0, 0, 1]])

        for i in range(mat.shape[0]-1):
            result[i] = transMat@rotMat@result[i]
        result[-1] = result[0]

        diff_coord = result[:, :-1] - mat

        updated_pos = {'BR': [diff_coord[0, 1], diff_coord[0, 0]],
                       'BL': [diff_coord[1, 1], diff_coord[1, 0]],
                       'FL': [diff_coord[2, 1], diff_coord[2, 0]],
                       'BODY': [diff_coord[3, 1], diff_coord[3, 0]],
                       'FR': [diff_coord[4, 1], diff_coord[4, 0]]}

        return updated_pos

    def timerCallback(self):

        if self.state == 0:
            self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
                self.response_body1.points[self.point_counter])

        elif self.state == 1:
            # self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg = self.response_leg.points[self.point_counter]
            # self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg = self.response_leg.points[self.point_counter]

            if self.first_step:
                self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = self.response_half_leg.points[self.point_counter]
                self.msg.body_leg_ik_trajectory[0].leg_points.back_right_leg = self.response_half_leg.points[self.point_counter]
            else:
                self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = self.response_minus_leg.points[self.point_counter]
                self.msg.body_leg_ik_trajectory[0].leg_points.back_right_leg = self.response_minus_leg.points[self.point_counter]

            # self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg.z += 0.005
            # self.msg.body_leg_ik_trajectory[0].leg_points.back_right_leg.z += 0.005

        elif self.state == 2:
            self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
                self.response_body2.points[self.point_counter])

        elif self.state == 3:
            self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg = self.response_leg.points[self.point_counter]
            self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg = self.response_leg.points[self.point_counter]

        # Add IMU control effort
        self.msg.body_leg_ik_trajectory[0].body_rotation = self.body_rotation

        # Publish
        self.ik_publisher_.publish(self.msg)

        # Update time and state counter
        if self.point_counter == self.gait_res - 1:
            if self.state == 3:
                
                self.gait_x_length = self.cmd_vel_msg.linear.x*(self.gait_period*4)
                self.gait_y_length = self.cmd_vel_msg.linear.y*(self.gait_period*4)
                self.gait_theta_length = self.cmd_vel_msg.angular.z*(self.gait_period*4)
                
                updated_pos = self.update_positions(
                                self.coord_orig, 
                                self.gait_x_length, 
                                self.gait_y_length, 
                                self.gait_theta_length)
                
                # reset whole gait
                self.msg.body_leg_ik_trajectory[0] = BodyLegIK()
                self.msg.body_leg_ik_trajectory[0].leg_points.reference_link = 1
                self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
                    Point(x=0.005, z=self.lower_body+0.002))
                self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = Point(
                    x=-0.02)
                self.msg.body_leg_ik_trajectory[0].leg_points.back_right_leg = Point(
                    x=-0.02)
                self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg.z = self.lower_leg
                self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg.z = self.lower_leg

                self.state = 0
            else:
                self.state += 1

            # Update trajectory
            if self.state == 0:
                future = self.traj_client.call_async(self.req_body1)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.response_body1 = future.result()
            elif self.state == 1:
                if self.first_step:
                    future = self.traj_client.call_async(self.req_half_leg)
                    rclpy.spin_until_future_complete(self.support_node, future)
                    self.response_half_leg = future.result()
                else:
                    future = self.traj_client.call_async(self.req_minus_leg)
                    rclpy.spin_until_future_complete(self.support_node, future)
                    self.response_minus_leg = future.result()

            elif self.state == 2:
                self.first_step = False
                future = self.traj_client.call_async(self.req_body2)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.response_body2 = future.result()
            elif self.state == 3:
                future = self.traj_client.call_async(self.req_leg)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.response_leg = future.result()

            # Update start_time and point_counter
            self.point_counter = 0
            self.start_time = time.time()

            time_delay = time.time() + 2
            while (time.time() < time_delay):
                continue
        else:
            self.point_counter += 1

    def point_to_vector3(self, point):
        return Vector3(x=point.x, y=point.y, z=point.z)


def main(args=None):
    rclpy.init(args=args)

    test_trot = TestTrot()

    # test_trot.get_logger().info("Gettings trajectories...")
    # future = test_trot.traj_client.call_async(test_trot.req_minus_leg)
    # rclpy.spin_until_future_complete(test_trot, future)
    # test_trot.response_minus_leg = future.result()

    # test_trot.get_logger().info("Gettings trajectories...")
    # future = test_trot.traj_client.call_async(test_trot.req_half_leg)
    # rclpy.spin_until_future_complete(test_trot, future)
    # test_trot.response_half_leg = future.result()

    # test_trot.get_logger().info("Gettings trajectories...")
    # future = test_trot.traj_client.call_async(test_trot.req_leg)
    # rclpy.spin_until_future_complete(test_trot, future)
    # test_trot.response_leg = future.result()

    future = test_trot.traj_client.call_async(test_trot.req_body1)
    rclpy.spin_until_future_complete(test_trot.support_node, future)
    test_trot.response_body1 = future.result()

    # future = test_trot.traj_client.call_async(test_trot.req_body2)
    # rclpy.spin_until_future_complete(test_trot, future)
    # test_trot.response_body2 = future.result()

    # print(len(test_trot.response_leg.points))
    # print(len(test_trot.response_body1.points))
    # print(len(test_trot.response_body2.points))

    test_trot.ik_timer.reset()

    while rclpy.ok():
        rclpy.spin_once(test_trot)

    test_trot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
