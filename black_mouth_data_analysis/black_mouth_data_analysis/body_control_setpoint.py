import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

import math

class BodyControlSetpoint(Node):

    def __init__(self):
        super().__init__('body_control_setpoint')

        self.imu_subscriber = self.create_subscription(Imu, 'imu/data', self.IMUCallback, 10)
        self.rotation_publisher = self.create_publisher(Vector3, 'body_rotation_euler', 10)
        self.setpoint_publisher = self.create_publisher(Vector3, 'body_desired_rotation', 10)
        self.publish_timer = self.create_timer(0.01, self.publishDesiredRotation)
        self.setpoint_timer = self.create_timer(3.0, self.changeSetpoint)

        self.rotation_msg = Vector3()
        self.setpoint_msg = Vector3()
        self.count = 0

    def IMUCallback(self, msg):
        self.rotation_msg.x, self.rotation_msg.y, self.rotation_msg.z = self.euler_from_quaternion(msg.orientation.x, 
                                                                                                   msg.orientation.y, 
                                                                                                   msg.orientation.z, 
                                                                                                   msg.orientation.w)
        self.rotation_publisher.publish(self.rotation_msg)

    def publishDesiredRotation(self):
        self.setpoint_publisher.publish(self.setpoint_msg)

    def changeSetpoint(self):
        if self.count < 3:
            self.setpoint_msg.x = 0.3
            self.setpoint_msg.y = 0.0
        elif self.count == 4:
            self.setpoint_msg.x = 0.0
            self.setpoint_msg.y = 0.3
        if self.count == 5:
            self.setpoint_msg.x = -0.3
            self.setpoint_msg.y = 0.0
        elif self.count == 6:
            self.setpoint_msg.x = 0.0
            self.setpoint_msg.y = -0.3
        elif self.count == 7:
            self.setpoint_msg.x = 0.2
            self.setpoint_msg.y = 0.2
        elif self.count == 8:
            self.setpoint_msg.x = -0.2
            self.setpoint_msg.y = -0.2
            self.count = -1
        self.count += 1

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

def main():
    rclpy.init()
    node = BodyControlSetpoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
