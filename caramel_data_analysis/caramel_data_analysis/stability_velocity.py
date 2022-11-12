#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter

import math
import time
import csv

class StabilityVelocity(Node):
    def __init__(self):
        super().__init__('stability_velocity')
      
        self.rotation_publisher = self.create_publisher(Vector3, 
                                                        'body_rotation_euler', 
                                                        10)
        self.vel_publisher = self.create_publisher(Twist, 
                                                  'cmd_vel', 
                                                  10)

        self.imu_subscriber = self.create_subscription(Imu, 
                                                      'imu/data', 
                                                      self.IMUCallback, 
                                                      10)
        
        self.use_imu_client = self.create_client(SetParameters,
                                                 'trot_gait_node/set_parameters')

        # self.vel_timer = self.create_timer(
        #    0.02, self.timerCallback)

        self.declare_parameter('use_stabilization', False)
        self.use_stabilization = self.get_parameter(
            'use_stabilization').get_parameter_value().bool_value

        self.rotation_msg = Vector3()
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.05

        self.max_pitch = 0.0
        self.min_pitch = 0.0
        self.max_roll = 0.0
        self.min_roll = 0.0

        while not self.use_imu_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('trot gait set parameters service not available, waiting again...')

        self.req = SetParameters.Request()
        param = Parameter()
        param.name = "use_imu"
        param.value.bool_value = self.use_stabilization
        self.req.parameters.append(param)

        self.future = self.use_imu_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        self.vel_publisher.publish(self.vel_msg)

    # def timerCallback(self):
    #     self.vel_publisher.publish(self.vel_msg)

    def IMUCallback(self, msg):
        self.rotation_msg.x, self.rotation_msg.y, self.rotation_msg.z = self.euler_from_quaternion(msg.orientation.x, 
                                                                                                   msg.orientation.y, 
                                                                                                   msg.orientation.z, 
                                                                                                   msg.orientation.w)
        self.rotation_publisher.publish(self.rotation_msg)
        if self.rotation_msg.x < self.min_roll:
            self.min_roll = self.rotation_msg.x
        elif self.rotation_msg.x > self.max_roll:
            self.max_roll = self.rotation_msg.x

        if self.rotation_msg.y < self.min_pitch:
            self.min_pitch = self.rotation_msg.y
        elif self.rotation_msg.y > self.max_pitch:
            self.max_pitch = self.rotation_msg.y

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
      
def rad2Degree(alpha):
    return alpha*180.0/math.pi

def main():
    rclpy.init()
    node = StabilityVelocity()

    initial_time = time.time()
    node.get_logger().info("INITIAL TIME")

    print("INITIAL TIME")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        last_time = time.time()
        elapsed_time = last_time - initial_time
        roll_oscilation = node.max_roll - node.min_roll
        pitch_oscilation = node.max_pitch - node.min_pitch

        print("FINAL:\n")
        print(f"\tElapsed time: {elapsed_time}")
        print(f"\tRoll: {rad2Degree(node.min_roll)} ~ {rad2Degree(node.max_roll)}, {rad2Degree(roll_oscilation)} ")
        print(f"\tPitch: {rad2Degree(node.min_pitch)} ~ {rad2Degree(node.max_pitch)}, {rad2Degree(pitch_oscilation)} ")

        with open('/home/ubuntu/bm_ws/src/bir-black-mouth/caramel_data_analysis/csv/stability_velocity.csv', 
                'a', newline='') as csv_file:
            fieldnames = ['estabilidade', 'tempo', 'roll_max', 'roll_min', 'roll_oscilation', 'pitch_max', 'pitch_min', 'pitch_oscilation']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writerow({'estabilidade': node.use_stabilization, 
                            'tempo': elapsed_time, 
                            'roll_max': rad2Degree(node.max_roll), 
                            'roll_min': rad2Degree(node.min_roll), 
                            'roll_oscilation': rad2Degree(roll_oscilation), 
                            'pitch_max': rad2Degree(node.max_pitch), 
                            'pitch_min': rad2Degree(node.min_pitch), 
                            'pitch_oscilation': rad2Degree(pitch_oscilation)})

    # node.vel_msg.linear.x = 0.0
    # node.vel_publisher.publish(node.vel_msg)
    # rclpy.spin_once(node)

    node.destroy_node()

    rclpy.shutdown()
