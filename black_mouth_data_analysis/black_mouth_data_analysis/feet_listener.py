import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

class FeetListener(Node):

    def __init__(self):
        super().__init__('feet_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.front_right_publisher = self.create_publisher(TransformStamped, 'feet_poses/front_right_foot', 1)
        self.front_left_publisher = self.create_publisher(TransformStamped, 'feet_poses/front_left_foot', 1)
        self.back_left_publisher = self.create_publisher(TransformStamped, 'feet_poses/back_left_foot', 1)
        self.back_right_publisher = self.create_publisher(TransformStamped, 'feet_poses/back_right_foot', 1)

        self.timer = self.create_timer(0.001, self.publishPoses)

    def publishPoses(self):
        
        try:
            front_right_transform = self.tf_buffer.lookup_transform("front_right_foot",
                                                                    "front_right_foot_default",
                                                                     rclpy.time.Time())
            front_left_transform = self.tf_buffer.lookup_transform("front_left_foot",
                                                                   "front_left_foot_default",
                                                                    rclpy.time.Time())
            back_left_transform = self.tf_buffer.lookup_transform("back_left_foot",
                                                                  "back_left_foot_default",
                                                                   rclpy.time.Time())
            back_right_transform = self.tf_buffer.lookup_transform("back_right_foot",
                                                                   "back_right_foot_default",
                                                                    rclpy.time.Time())
            self.front_right_publisher.publish(front_right_transform)
            self.front_left_publisher.publish(front_left_transform)
            self.back_left_publisher.publish(back_left_transform)
            self.back_right_publisher.publish(back_right_transform)
        except TransformException as ex:
                self.get_logger().info(f'Could not transform: {ex}')
                return


def main():
    rclpy.init()
    node = FeetListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
