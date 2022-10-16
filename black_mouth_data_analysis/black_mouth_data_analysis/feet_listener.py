import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import Pose, PoseArray

class FeetListener(Node):

    def __init__(self):
        super().__init__('feet_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.publisher = self.create_publisher(PoseArray, 'feet_poses', 1)
        self.timer = self.create_timer(0.001, self.publishPoses)

    def publishPoses(self):
        feet_poses = PoseArray()
        feet_poses.poses = [Pose()]*4
        for i, foot in enumerate(["front_right_foot", "front_left_foot", "back_left_foot", "back_right_foot"]):
            try:
                transform = self.tf_buffer.lookup_transform(foot,
                                                            foot+"_default",
                                                            rclpy.time.Time())
                feet_poses.poses[i].position.x = transform.transform.translation.x
                feet_poses.poses[i].position.y = transform.transform.translation.y
                feet_poses.poses[i].position.z = transform.transform.translation.z
                feet_poses.poses[i].orientation = transform.transform.rotation
            except TransformException as ex:
                    self.get_logger().info(f'Could not transform {foot} to {foot}_default: {ex}')
                    return
        self.publisher.publish(feet_poses)


def main():
    rclpy.init()
    node = FeetListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
