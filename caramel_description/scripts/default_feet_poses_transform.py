#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import Point, Vector3
from caramel_kinematics.msg import AllLegPoints

from time import time

class FeetListener(Node):

    def __init__(self):
        super().__init__('feet_listener')

        self.get_logger().info('feet_listener node initialized')

        self.time = time()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_pub = self.create_publisher(AllLegPoints, 'feet_poses', 1)

        self.timer = self.create_timer(0.01, self.publishPoses)

    def publishPoses(self):
        
        try:
            front_right_transform = self.tf_buffer.lookup_transform("front_right_foot_default",
                                                                    "front_right_foot",
                                                                     rclpy.time.Time())
            front_left_transform = self.tf_buffer.lookup_transform("front_left_foot_default",
                                                                   "front_left_foot",
                                                                    rclpy.time.Time())
            back_left_transform = self.tf_buffer.lookup_transform("back_left_foot_default",
                                                                  "back_left_foot",
                                                                   rclpy.time.Time())
            back_right_transform = self.tf_buffer.lookup_transform("back_right_foot_default",
                                                                   "back_right_foot",
                                                                    rclpy.time.Time())

            msg = AllLegPoints()
            msg.front_right_leg = self.vector3_to_point(front_right_transform.transform.translation)
            msg.front_left_leg = self.vector3_to_point(front_left_transform.transform.translation)
            msg.back_left_leg = self.vector3_to_point(back_left_transform.transform.translation)
            msg.back_right_leg = self.vector3_to_point(back_right_transform.transform.translation)
            self.tf_pub.publish(msg)

        except TransformException as ex:
            if time() - self.time > 1.0:
                self.time = time()
                self.get_logger().info(f'Could not transform: {ex}')
            return

    def point_to_vector3(self, point):
        return Vector3(x=point.x, y=point.y, z=point.z)

    def vector3_to_point(self, vector):
        return Point(x=vector.x, y=vector.y, z=vector.z)


def main():
    rclpy.init()
    node = FeetListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__=="__main__":
    main()
