#!/usr/bin/env python3
import rclpy
import time
import numpy as np
import math
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Vector3, Point, Twist
from sensor_msgs.msg import Imu
from caramel_control.msg import BodyControl
from caramel_kinematics.msg import BodyLegIKTrajectory, BodyLegIK, AllLegPoints
from caramel_gait_planner.srv import ComputeGaitTrajectory
from caramel_teleop.msg import TeleopState


class TrotGait(Node):
    def __init__(self):
        super().__init__('trot_gait_node')

        self.declare_parameter('use_imu', False)
        self.declare_parameter('gait_period', 0.5)
        self.declare_parameter('gait_height', 0.03)
        self.declare_parameter('ground_penetration', 0.03)
        self.declare_parameter('fixed_forward_body', 0.02)
        self.declare_parameter('body_height', 0.0)
        self.declare_parameter('feet_y_inside', 0.0)
        self.declare_parameter('resolution_first_fraction', 0.33)
        self.declare_parameter('period_first_fraction', 0.66)
        self.declare_parameters('adjust_feet_height', [("front_right", 0.0),
                                                       ("front_left", 0.0),
                                                       ("back_left", 0.0),
                                                       ("back_right", 0.0)])

        self.use_imu = self.get_parameter(
            'use_imu').get_parameter_value().bool_value
        self.gait_period = self.get_parameter(
            'gait_period').get_parameter_value().double_value
        self.gait_height = self.get_parameter(
            'gait_height').get_parameter_value().double_value
        self.ground_penetration = self.get_parameter(
            'ground_penetration').get_parameter_value().double_value
        self.fixed_forward_body = self.get_parameter(
            'fixed_forward_body').get_parameter_value().double_value
        self.body_height = self.get_parameter(
            'body_height').get_parameter_value().double_value
        self.feet_y_inside = self.get_parameter(
            'feet_y_inside').get_parameter_value().double_value
        self.resolution_first_fraction = self.get_parameter(
            'resolution_first_fraction').get_parameter_value().double_value
        self.period_first_fraction = self.get_parameter(
            'period_first_fraction').get_parameter_value().double_value
        
        self.adjust_feet_height = self.get_parameters(['adjust_feet_height.front_right',
                                                       'adjust_feet_height.front_left',
                                                       'adjust_feet_height.back_left',
                                                       'adjust_feet_height.back_right'])

        self.adjust_feet = dict()
        self.adjust_feet['FR'] = self.adjust_feet_height[0].get_parameter_value().double_value
        self.adjust_feet['FL'] = self.adjust_feet_height[1].get_parameter_value().double_value
        self.adjust_feet['BL'] = self.adjust_feet_height[2].get_parameter_value().double_value
        self.adjust_feet['BR'] = self.adjust_feet_height[3].get_parameter_value().double_value

        self.update_params = False
        self.add_on_set_parameters_callback(self.parametersCallback)

        self.body_imu_rotation = Vector3()

        self.ik_publisher_ = self.create_publisher(BodyLegIKTrajectory,
                                                   '/cmd_ik',
                                                   10)

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


        self.default_feet_pose_subscriber = self.create_subscription(AllLegPoints,
                                                                     'feet_poses',
                                                                     self.default_feet_points,
                                                                     10)
        self.default_feet_pose_msg = AllLegPoints()


        self.current_state_msg = TeleopState()

        self.gait_x_length = 0
        self.gait_theta_length = 0
        self.gait_y_length = 0

        self.cmd_vel_msg = Twist()

        self.timer_period = 0.02
        self.ik_timer = self.create_timer(
            self.timer_period, self.timerCallback)
        self.ik_timer.cancel()

        self.gait_res = int(self.gait_period/self.timer_period)

        self.start_time = time.time()
        self.point_counter = 0

        self.support_node = Node('trot_gait_node_support')
        
        self.traj_client = self.support_node.create_client(
            ComputeGaitTrajectory, 'compute_gait_trajectory')
        while not self.traj_client.wait_for_service(1.0):
            if not rclpy.ok():
                self.get_logger().error(
                    f"Interrupted while waiting for the {self.traj_client.srv_name} service. Exiting.")
                return
            self.get_logger().info(
                f"{self.traj_client.srv_name} service not available, waiting...")

        self.ik_server_params_client = self.support_node.create_client(
            GetParameters, 'compute_ik_server/get_parameters')
        while not self.ik_server_params_client.wait_for_service(1.0):
            if not rclpy.ok():
                self.get_logger().error(
                    f"Interrupted while waiting for the {self.ik_server_params_client.srv_name} service. Exiting.")
                return
            self.get_logger().info(
                f"{self.ik_server_params_client.srv_name} service not available, waiting...")

        self.Length, self.Width, self.L1 = self.getIKParams()

        self.last_step_l = 0.0
        self.create_body_matrix()
        self.update_positions(
            self.coord_orig,
            self.gait_x_length,
            self.gait_y_length,
            self.gait_theta_length)

        # ---------------------------------------------------------------------------
        self.FL_request = ComputeGaitTrajectory.Request()
        self.FL_request.initial_point = Point(z=self.adjust_feet['FL'],
                                              y=-self.feet_y_inside)
        self.FL_request.landing_point = Point(x=self.updated_pos['FL'][0],
                                              y=self.updated_pos['FL'][1]-self.feet_y_inside,
                                              z=self.adjust_feet['FL'])
        self.FL_request.period = self.gait_period
        self.FL_request.height = self.gait_height
        self.FL_request.resolution = self.gait_res
        self.FL_request.resolution_first_fraction = self.resolution_first_fraction
        self.FL_request.period_first_fraction = self.period_first_fraction
        self.FL_response = None
        # ---------------------------------------------------------------------------
        self.FR_request = ComputeGaitTrajectory.Request()
        self.FR_request.initial_point = Point(z=self.adjust_feet['FR'],
                                              y=self.feet_y_inside)
        self.FR_request.landing_point = Point(x=self.updated_pos['FR'][0],
                                              y=self.updated_pos['FR'][1]+self.feet_y_inside,
                                              z=self.adjust_feet['FR'])
        self.FR_request.period = self.gait_period
        self.FR_request.height = self.gait_height
        self.FR_request.resolution = self.gait_res
        self.FR_request.resolution_first_fraction = self.resolution_first_fraction
        self.FR_request.period_first_fraction = self.period_first_fraction
        self.FR_response = None
        # ---------------------------------------------------------------------------
        self.BL_request = ComputeGaitTrajectory.Request()
        self.BL_request.initial_point = Point(z=self.adjust_feet['BL'],
                                              y=-self.feet_y_inside)
        self.BL_request.landing_point = Point(x=self.updated_pos['BL'][0],
                                              y=self.updated_pos['BL'][1]-self.feet_y_inside,
                                              z=self.adjust_feet['BL'])
        self.BL_request.period = self.gait_period
        self.BL_request.height = self.gait_height
        self.BL_request.resolution = self.gait_res
        self.BL_request.resolution_first_fraction = self.resolution_first_fraction
        self.BL_request.period_first_fraction = self.period_first_fraction
        self.BL_response = None
        # ---------------------------------------------------------------------------
        self.BR_request = ComputeGaitTrajectory.Request()
        self.BR_request.initial_point = Point(z=self.adjust_feet['BR'],
                                              y=self.feet_y_inside)
        self.BR_request.landing_point = Point(x=self.updated_pos['BR'][0],
                                              y=self.updated_pos['BR'][1]+self.feet_y_inside,
                                              z=self.adjust_feet['BR'])
        self.BR_request.period = self.gait_period
        self.BR_request.height = self.gait_height
        self.BR_request.resolution = self.gait_res
        self.BR_request.resolution_first_fraction = self.resolution_first_fraction
        self.BR_request.period_first_fraction = self.period_first_fraction
        self.BR_response = None
        # ---------------------------------------------------------------------------
        self.BODY_request = ComputeGaitTrajectory.Request()
        self.BODY_request.initial_point = Point(
            x=self.fixed_forward_body, z=self.body_height)
        self.BODY_request.landing_point = Point(x=self.updated_pos['BODY'][0]+self.fixed_forward_body,
                                                y=self.updated_pos['BODY'][1],
                                                z=self.body_height)
        self.BODY_request.period = self.gait_period
        self.BODY_request.height = self.ground_penetration
        self.BODY_request.resolution = self.gait_res
        self.BODY_request.resolution_first_fraction = 1.0
        self.BODY_request.period_first_fraction = 1.0
        self.BODY_response = None
        self.BODY_rotation = np.array([])
        self.progress_time_vector = np.array([])
        # ---------------------------------------------------------------------------

        self.state = 0
        self.walking = 0.0
        self.start_time = 0.0
        self.point_counter = 0

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
        self.msg.time_from_start.append(Duration())

        self.get_logger().info("Ready to walk!", once=True)

    def updateGaitParams(self):
        self.gait_res = int(self.gait_period/self.timer_period)

        self.FL_request.period = self.gait_period
        self.FL_request.height = self.gait_height
        self.FL_request.resolution = self.gait_res

        self.FR_request.period = self.gait_period
        self.FR_request.height = self.gait_height
        self.FR_request.resolution = self.gait_res

        self.BL_request.period = self.gait_period
        self.BL_request.height = self.gait_height
        self.BL_request.resolution = self.gait_res

        self.BR_request.period = self.gait_period
        self.BR_request.height = self.gait_height
        self.BR_request.resolution = self.gait_res

        self.BODY_request.period = self.gait_period
        self.BODY_request.height = self.ground_penetration
        self.BODY_request.resolution = self.gait_res

    def parametersCallback(self, params):
        for param in params:
            if param.name == "gait_period":
                self.gait_period = param.value
            elif param.name == "gait_height":
                self.gait_height = param.value
            elif param.name == "use_imu":
                self.use_imu = param.value
            elif param.name == "ground_penetration":
                self.ground_penetration = param.value
            self.get_logger().info(param.name + " set to " + str(param.value))

        self.update_params = True
        return SetParametersResult(successful=True)

    def getIKParams(self):
        ik_params_req = GetParameters.Request()
        ik_params_req.names = ["body_dimensions.L", "body_dimensions.W", "leg_dimensions.L1"]
        
        future = self.ik_server_params_client.call_async(ik_params_req)
        rclpy.spin_until_future_complete(self.support_node, future)
        params = future.result()

        body_length = params.values[0].double_value
        body_width = params.values[1].double_value
        leg_l1 = params.values[2].double_value
        
        return body_length, body_width, leg_l1


    def create_body_matrix(self):
        self.coord_orig = np.array([[0.0, 0.0],
                                    [self.Length/2 + self.default_feet_pose_msg.front_right_leg.x + self.fixed_forward_body, 
                                     -(self.Width/2 + self.L1) + self.default_feet_pose_msg.front_right_leg.y],
                                    [self.Length/2 + self.default_feet_pose_msg.front_left_leg.x + self.fixed_forward_body,
                                     +(self.Width/2 + self.L1) + self.default_feet_pose_msg.front_left_leg.y],
                                    [-self.Length/2 + self.default_feet_pose_msg.back_left_leg.x + self.fixed_forward_body,
                                     +(self.Width/2 + self.L1) + self.default_feet_pose_msg.back_left_leg.y],
                                    [-self.Length/2 + self.default_feet_pose_msg.back_right_leg.x + self.fixed_forward_body,
                                     -(self.Width/2 + self.L1) + self.default_feet_pose_msg.back_right_leg.y]])
        self.coord_orig = np.vstack((self.coord_orig, self.coord_orig[0]))

    def bodyCallback(self, msg):
        if self.use_imu:
            self.body_imu_rotation = msg.body_rotation
        else:
            self.body_imu_rotation = Vector3()

        
    def default_feet_points(self, msg):
        self.default_feet_pose_msg = msg

    def euler_from_quaternion(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


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

        rotMat = np.array([[np.cos(theta), -1*np.sin(theta), 0],
                           [np.sin(theta), np.cos(theta), 0],
                           [0, 0, 1]])

        x = linear_x*np.cos(theta) - linear_y*np.sin(theta)
        y = linear_x*np.sin(theta) + linear_y*np.cos(theta)

        transMat = np.array([[1, 0, x],
                             [0, 1, y],
                             [0, 0, 1]])

        for i in range(mat.shape[0]-1):
            result[i] = transMat@rotMat@result[i]
        result[-1] = result[0]


        diff_coord = result[:, :-1] - self.coord_orig
        # print(diff_coord, '\n')

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
                    self.walking = 0.0
                    if self.current_state_msg.state != 5 and self.current_state_msg.state != 1:
                        self.ik_timer.cancel()
                else:
                    self.FL_request.height = self.gait_height
                    self.FR_request.height = self.gait_height
                    self.BL_request.height = self.gait_height
                    self.BR_request.height = self.gait_height
                    self.BODY_request.height = self.ground_penetration
                    self.walking = 1.0

                self.msg.body_leg_ik_trajectory[0] = BodyLegIK()
                self.msg.body_leg_ik_trajectory[0].leg_points.reference_link = 1
                self.msg.body_leg_ik_trajectory[0].body_position = self.point_to_vector3(
                    Point(x=self.fixed_forward_body - self.last_step_l/2, z=self.body_height))
                self.msg.body_leg_ik_trajectory[0].leg_points.front_right_leg = Point(
                    x=(self.default_feet_pose_msg.front_right_leg.x + self.fixed_forward_body)*self.walking - self.last_step_l/2,
                    y=self.default_feet_pose_msg.front_right_leg.y,
                    z=self.adjust_feet['FR'])
                self.msg.body_leg_ik_trajectory[0].leg_points.front_left_leg = Point(
                    y=-self.feet_y_inside,
                    z=self.adjust_feet['FL'])
                self.msg.body_leg_ik_trajectory[0].leg_points.back_left_leg = Point(
                    x=(self.default_feet_pose_msg.back_left_leg.x + self.fixed_forward_body)*self.walking - self.last_step_l/2,
                    y=self.default_feet_pose_msg.back_left_leg.y,
                    z=self.adjust_feet['BL'])
                self.msg.body_leg_ik_trajectory[0].leg_points.back_right_leg = Point(
                    y=self.feet_y_inside,
                    z=self.adjust_feet['BR'])

                if self.update_params:
                    self.updateGaitParams()
                    self.update_params = False

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
                self.progress_time_vector = np.array(
                    [t.sec + t.nanosec*1e-9 for t in self.BODY_response.time_from_start]
                ) / self.gait_period
                self.BODY_rotation = self.gait_theta_length/2 * self.progress_time_vector

            elif self.state == 1:
                self.FR_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.front_right_leg
                self.FR_request.landing_point.x = self.updated_pos['FR'][0] / 2
                self.FR_request.landing_point.y = self.updated_pos['FR'][1] + self.feet_y_inside
                future = self.traj_client.call_async(self.FR_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.FR_response = future.result()

                self.BL_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.back_left_leg
                self.BL_request.landing_point.x = self.updated_pos['BL'][0] / 2
                self.BL_request.landing_point.y = self.updated_pos['BL'][1] - self.feet_y_inside
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
                self.BODY_rotation = (self.gait_theta_length/2 *
                                      self.progress_time_vector) + self.gait_theta_length/2

            elif self.state == 3:
                self.FL_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.front_left_leg
                self.FL_request.landing_point.x = self.updated_pos['FL'][0]
                self.FL_request.landing_point.y = self.updated_pos['FL'][1] - self.feet_y_inside
                future = self.traj_client.call_async(self.FL_request)
                rclpy.spin_until_future_complete(self.support_node, future)
                self.FL_response = future.result()

                self.BR_request.initial_point = self.msg.body_leg_ik_trajectory[
                    0].leg_points.back_right_leg
                self.BR_request.landing_point.x = self.updated_pos['BR'][0]
                self.BR_request.landing_point.y = self.updated_pos['BR'][1] + self.feet_y_inside
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

    trot_gait = TrotGait()

    # Get first body trajectory
    future = trot_gait.traj_client.call_async(trot_gait.BODY_request)
    rclpy.spin_until_future_complete(trot_gait.support_node, future)
    trot_gait.BODY_response = future.result()
    trot_gait.progress_time_vector = np.array(
        [t.sec + t.nanosec*1e-9 for t in trot_gait.BODY_response.time_from_start]) / trot_gait.gait_period
    trot_gait.BODY_rotation = -trot_gait.gait_theta_length / \
        2 * trot_gait.progress_time_vector

    # trot_gait.ik_timer.reset()
    rclpy.spin(trot_gait)

    trot_gait.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
