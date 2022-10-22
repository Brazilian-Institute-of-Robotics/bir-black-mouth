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
from black_mouth_teleop.msg import TeleopState


class TestTrot(Node):
    def __init__(self):
        super().__init__('test_trot_node')

        self.declare_parameter('use_imu', False)

        self.body_imu_rotation = Vector3()

        self.ik_publisher_ = self.create_publisher(
            BodyLegIKTrajectory, '/cmd_ik', 10)
        self.body_control_subscriber = self.create_subscription(BodyControl,
                                                                '/body_control',
                                                                self.bodyCallback,
                                                                10)

        self.cmd_vel_subscriber = self.create_subscription(Twist,
                                                           '/cmd_vel',
                                                           self.cmd_vel_cb,
                                                           10)

        self.current_state_subscriber = self.create_subscription(TeleopState,
                                                                 '/teleop_state',
                                                                 self.current_state_cb,
                                                                 10)
        self.current_state_msg = TeleopState()

        self.gait_x_length = 0
        self.gait_theta_length = 0
        self.gait_y_length = 0

        self.fixed_forward_body = 0.02

        self.use_imu = False

        self.cmd_vel_msg = Twist()

        timer_period = 0.02
        self.ik_timer = self.create_timer(timer_period, self.timerCallback)
        self.ik_timer.cancel()

        self.start_time = time.time()
        self.point_counter = 0

        self.support_node = Node('test_trot_node_support')
        self.traj_client = self.support_node.create_client(
            ComputeGaitTrajectory, 'compute_gait_trajectory')

        self.get_logger().info("Waiting for server")
        while not self.traj_client.wait_for_service(1.0):
            self.get_logger().info("...", once=True)

        self.gait_period = 0.5  # secs
        self.gait_res = int(self.gait_period/timer_period)  # secs
        # self.gait_res = 7  # secs

        self.lower_body = 0.002
        self.forward_body = 0.0
        self.lower_leg = -0.0045
        self.gait_height = 0.03
        self.resolution_first_fraction = 0.33
        self.period_first_fraction = 0.66

        self.Length = 0.2291
        self.Width = 0.140
        self.L1 = 0.048
        self.origin = 0.0
        self.last_step_l = 0.0
        self.create_body_matrix()
        self.update_positions(
            self.coord_orig,
            self.gait_x_length,
            self.gait_y_length,
            self.gait_theta_length)

        # ---------------------------------------------------------------------------
        self.FL_request = ComputeGaitTrajectory.Request()
        self.FL_request.initial_point = Point()
        self.FL_request.landing_point = Point(x=self.updated_pos['FL'][0],
                                              y=self.updated_pos['FL'][1])
        self.FL_request.period = self.gait_period
        self.FL_request.height = self.gait_height
        self.FL_request.resolution = self.gait_res
        self.FL_request.resolution_first_fraction = self.resolution_first_fraction
        self.FL_request.period_first_fraction = self.period_first_fraction
        self.FL_response = None
        # ---------------------------------------------------------------------------
        self.FR_request = ComputeGaitTrajectory.Request()
        self.FR_request.initial_point = Point(z=self.lower_leg)
        self.FR_request.landing_point = Point(x=self.updated_pos['FR'][0],
                                              y=self.updated_pos['FR'][1],
                                              z=self.lower_leg)
        self.FR_request.period = self.gait_period
        self.FR_request.height = self.gait_height
        self.FR_request.resolution = self.gait_res
        self.FR_request.resolution_first_fraction = self.resolution_first_fraction
        self.FR_request.period_first_fraction = self.period_first_fraction
        self.FR_response = None
        # ---------------------------------------------------------------------------
        self.BL_request = ComputeGaitTrajectory.Request()
        self.BL_request.initial_point = Point(z=self.lower_leg)
        self.BL_request.landing_point = Point(x=self.updated_pos['BL'][0],
                                              y=self.updated_pos['BL'][1],
                                              z=self.lower_leg)
        self.BL_request.period = self.gait_period
        self.BL_request.height = self.gait_height
        self.BL_request.resolution = self.gait_res
        self.BL_request.resolution_first_fraction = self.resolution_first_fraction
        self.BL_request.period_first_fraction = self.period_first_fraction
        self.BL_response = None
        # ---------------------------------------------------------------------------
        self.BR_request = ComputeGaitTrajectory.Request()
        self.BR_request.initial_point = Point()
        self.BR_request.landing_point = Point(x=self.updated_pos['BR'][0],
                                              y=self.updated_pos['BR'][1])
        self.BR_request.period = self.gait_period
        self.BR_request.height = self.gait_height
        self.BR_request.resolution = self.gait_res
        self.BR_request.resolution_first_fraction = self.resolution_first_fraction
        self.BR_request.period_first_fraction = self.period_first_fraction
        self.BR_response = None
        # ---------------------------------------------------------------------------
        self.BODY_request = ComputeGaitTrajectory.Request()
        self.BODY_request.initial_point = Point(
            x=self.forward_body, z=self.lower_body)
        self.BODY_request.landing_point = Point(x=self.updated_pos['BODY'][0]+self.forward_body,
                                                y=self.updated_pos['BODY'][1],
                                                z=self.lower_body)
        self.BODY_request.period = self.gait_period
        self.BODY_request.height = 0.005  # Ground penetration height
        self.BODY_request.resolution = self.gait_res
        self.BODY_request.resolution_first_fraction = 1.0
        self.BODY_request.period_first_fraction = 1.0
        self.BODY_response = None
        self.BODY_rotation = np.array([])
        self.progress_time_vector = np.array([])
        # ---------------------------------------------------------------------------

        self.state = 0
        self.start_time = 0.0
        self.point_counter = 0
        self.t1 = Duration(sec=0, nanosec=int(timer_period*1e9))

        # Set initial position
        self.msg = BodyLegIKTrajectory()
        self.msg.body_leg_ik_trajectory.append(BodyLegIK())
        self.msg.body_leg_ik_trajectory[0].leg_points.reference_link = 1
        self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg = self.FR_request.initial_point
        self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = self.FL_request.initial_point
        self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg = self.BL_request.initial_point
        self.msg.body_leg_ik_trajectory[0].leg_points.back_right_leg = self.BR_request.initial_point
        self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
            self.BODY_request.initial_point)
        self.msg.time_from_start.append(self.t1)

        self.get_logger().info("Ready to walk!", once=True)

    def create_body_matrix(self):
        self.coord_orig = np.array([[0.0, 0.0],
                                    [self.Length/2 - self.last_step_l /
                                     2, -(self.Width/2 + self.L1)],
                                    [self.Length/2, +(self.Width/2 + self.L1)],
                                    [-self.Length/2 - self.last_step_l /
                                     2, +(self.Width/2 + self.L1)],
                                    [-self.Length/2, -(self.Width/2 + self.L1)]])
        self.coord_orig = np.vstack((self.coord_orig, self.coord_orig[0]))

    def bodyCallback(self, msg):
        if self.get_parameter('use_imu'):
            self.body_imu_rotation = msg.body_rotation

    def cmd_vel_cb(self, msg):
        self.cmd_vel_msg = msg

    def current_state_cb(self, msg):
        # if transitioning to WALKING state
        if self.current_state_msg.state != 5 and msg.state == 5:
            self.ik_timer.reset()
        self.current_state_msg = msg

    def update_positions(self, mat, linear_x, linear_y,  theta):
        result = np.ones((mat.shape[0], 3))
        result[:, :-1] = mat

        rotMat = np.array([[np.cos(-theta), -1*np.sin(-theta), 0],
                           [np.sin(-theta), np.cos(-theta), 0],
                           [0, 0, 1]])

        x = linear_x*np.cos(theta) - linear_y*np.sin(theta)
        y = linear_x*np.sin(theta) + linear_y*np.cos(theta)

        transMat = np.array([[1, 0, x],
                            [0, 1, y],
                            [0, 0, 1]])

        for i in range(mat.shape[0]-1):
            result[i] = transMat@rotMat@result[i]
        result[-1] = result[0]

        # TODO check with kinematics
        # diff_coord = np.zeros((5, 2))
        # diff_coord[0] = result[0, :-1] - mat[0, :-1]  # FR
        # diff_coord[1] = result[1, :-1] - mat[1, :-1]  # FL
        # diff_coord[2] = result[2, :-1] - mat[2, :-1]  # BL
        # diff_coord[3] = result[3, :-1] - mat[3, :-1]  # BODY
        # diff_coord[4] = result[4, :-1] - mat[4, :-1]  # BR

        diff_coord = result[:, :-1] - self.coord_orig
        print(diff_coord, '\n')

        self.updated_pos = {'BODY': [diff_coord[0, 0], diff_coord[0, 1]],
                            'FR': [diff_coord[1, 0], diff_coord[1, 1]],
                            'FL': [diff_coord[2, 0], diff_coord[2, 1]],
                            'BL': [diff_coord[3, 0], diff_coord[3, 1]],
                            'BR': [diff_coord[4, 0], diff_coord[4, 1]]}

    def timerCallback(self):

        if self.state == 0:
            self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
                self.BODY_response.points[self.point_counter])
            self.msg.body_leg_ik_trajectory[0].body_rotation.z = self.BODY_rotation[self.point_counter]

        elif self.state == 1:
            self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg = self.FR_response.points[self.point_counter]
            self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg = self.BL_response.points[self.point_counter]

        elif self.state == 2:
            self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
                self.BODY_response.points[self.point_counter])
            self.msg.body_leg_ik_trajectory[0].body_rotation.z = self.BODY_rotation[self.point_counter]

        elif self.state == 3:
            self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = self.FL_response.points[self.point_counter]
            self.msg.body_leg_ik_trajectory[0].leg_points.back_right_leg = self.BR_response.points[self.point_counter]

        # Add IMU control effort
        self.msg.body_leg_ik_trajectory[0].body_rotation.x = self.body_imu_rotation.x
        self.msg.body_leg_ik_trajectory[0].body_rotation.y = self.body_imu_rotation.y

        # Publish
        self.ik_publisher_.publish(self.msg)

        # Update time and state counter
        if self.point_counter == self.gait_res - 1:
            if self.state == 3:
                # Update coord matrix
                self.last_step_l = self.gait_x_length
                self.create_body_matrix()

                if self.last_step_l == 0.0 and self.cmd_vel_msg.linear.y == 0.0 and self.cmd_vel_msg.linear.x == 0.0 and self.cmd_vel_msg.angular.z == 0.0:
                    self.FL_request.height = 0.0
                    self.FR_request.height = 0.0
                    self.BL_request.height = 0.0
                    self.BR_request.height = 0.0
                    self.BODY_request.height = 0.0

                    if self.current_state_msg.state != 5:
                        self.ik_timer.cancel()
                else:
                    self.FL_request.height = self.gait_height
                    self.FR_request.height = self.gait_height
                    self.BL_request.height = self.gait_height
                    self.BR_request.height = self.gait_height
                    self.BODY_request.height = 0.005

                # Reset whole gait
                self.msg.body_leg_ik_trajectory[0] = BodyLegIK()
                self.msg.body_leg_ik_trajectory[0].leg_points.reference_link = 1
                self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
                    Point(x=self.fixed_forward_body - self.last_step_l/2, z=self.lower_body))
                self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg = Point(
                    x=-self.last_step_l/2, z=self.lower_leg)
                self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg = Point(
                    x=-self.last_step_l/2, z=self.lower_leg)

                # Get new X, Y, Yaw values
                self.gait_x_length = self.cmd_vel_msg.linear.x * \
                    (self.gait_period*4)
                self.gait_y_length = self.cmd_vel_msg.linear.y * \
                    (self.gait_period*4)
                self.gait_theta_length = self.cmd_vel_msg.angular.z * \
                    (self.gait_period*4)

                # Get updated positions
                self.update_positions(
                    self.coord_orig,
                    self.gait_x_length,
                    self.gait_y_length,
                    self.gait_theta_length)

                self.state = 0
            else:
                self.state += 1

            # Update trajectory
            if self.state == 0:
                self.BODY_request.initial_point = self.vector3_to_point(
                    self.msg.body_leg_ik_trajectory[0].body_position)
                self.BODY_request.landing_point.x = self.fixed_forward_body
                self.BODY_request.landing_point.y = self.BODY_request.initial_point.y + \
                    self.updated_pos['BODY'][1] / 2
                future = self.traj_client.call_async(self.BODY_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.BODY_response = future.result()
                self.BODY_rotation = -self.gait_theta_length/2 * self.progress_time_vector

            elif self.state == 1:
                self.FR_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.front_right_leg
                self.FR_request.landing_point.x = self.updated_pos['FR'][0] / 2
                self.FR_request.landing_point.y = self.updated_pos['FR'][1]
                future = self.traj_client.call_async(self.FR_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.FR_response = future.result()

                self.BL_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.back_left_leg
                self.BL_request.landing_point.x = self.updated_pos['BL'][0] / 2
                self.BL_request.landing_point.y = self.updated_pos['BL'][1]
                future = self.traj_client.call_async(self.BL_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.BL_response = future.result()

            elif self.state == 2:
                self.BODY_request.initial_point = self.vector3_to_point(
                    self.msg.body_leg_ik_trajectory[0].body_position)
                self.BODY_request.landing_point.x = self.BODY_request.initial_point.x + \
                    self.updated_pos['BODY'][0] / 2
                self.BODY_request.landing_point.y = self.BODY_request.initial_point.y + \
                    self.updated_pos['BODY'][1] / 2
                future = self.traj_client.call_async(self.BODY_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.BODY_response = future.result()
                self.BODY_rotation = -1*((self.gait_theta_length/2 *
                                      self.progress_time_vector) + self.gait_theta_length/2)

            elif self.state == 3:
                self.FL_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.front_left_leg
                self.FL_request.landing_point.x = self.updated_pos['FL'][0]
                self.FL_request.landing_point.y = self.updated_pos['FL'][1]
                future = self.traj_client.call_async(self.FL_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.FL_response = future.result()

                self.BR_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.back_right_leg
                self.BR_request.landing_point.x = self.updated_pos['BR'][0]
                self.BR_request.landing_point.y = self.updated_pos['BR'][1]
                future = self.traj_client.call_async(self.BR_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.BR_response = future.result()

            # Update start_time and point_counter
            self.point_counter = 0
            self.start_time = time.time()

            # time_delay = time.time() + 3
            # while (time.time() < time_delay):
            #     continue
        else:
            self.point_counter += 1

    def point_to_vector3(self, point):
        return Vector3(x=point.x, y=point.y, z=point.z)

    def vector3_to_point(self, vector):
        return Point(x=vector.x, y=vector.y, z=vector.z)


def main(args=None):
    rclpy.init(args=args)

    test_trot = TestTrot()

    # Get first body trajectory
    future = test_trot.traj_client.call_async(test_trot.BODY_request)
    rclpy.spin_until_future_complete(test_trot.support_node, future)
    test_trot.BODY_response = future.result()
    test_trot.progress_time_vector = np.array(
        [t.sec + t.nanosec*1e-9 for t in test_trot.BODY_response.time_from_start]) / test_trot.gait_period
    test_trot.BODY_rotation = -test_trot.gait_theta_length / \
        2 * test_trot.progress_time_vector

    # test_trot.ik_timer.reset()
    rclpy.spin(test_trot)

    test_trot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
