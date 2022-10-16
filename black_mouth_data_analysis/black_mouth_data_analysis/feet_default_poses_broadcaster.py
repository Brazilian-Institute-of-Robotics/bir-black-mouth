import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class FeetDefaultPosesFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('feet_default_poses_broadcaster')
        self.get_logger().info("Feet default poses frame broadcaster initialized")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.001, self.broadcast_timer_callback)

        self.body_dimensions = {"L": 0.2291, "W": 0.140, "H": 0.170}
        self.leg_dimensions = {"L1": 0.048, "L2": 0.120, "L3": 0.120}

    def broadcast_timer_callback(self):
        front_right_foot = TransformStamped()
        front_left_foot = TransformStamped()
        back_left_foot = TransformStamped()
        back_right_foot = TransformStamped()

        front_right_foot.header.stamp = self.get_clock().now().to_msg()
        front_right_foot.header.frame_id = 'base_link'
        front_right_foot.child_frame_id = 'front_right_foot_default'
        front_right_foot.transform.translation.x = self.body_dimensions["L"]/2
        front_right_foot.transform.translation.y = -(self.body_dimensions["W"]/2+self.leg_dimensions["L1"])
        front_right_foot.transform.translation.z = -self.body_dimensions["H"]

        front_left_foot.header.stamp = self.get_clock().now().to_msg()
        front_left_foot.header.frame_id = 'base_link'
        front_left_foot.child_frame_id = 'front_left_foot_default'
        front_left_foot.transform.translation.x = self.body_dimensions["L"]/2
        front_left_foot.transform.translation.y = (self.body_dimensions["W"]/2+self.leg_dimensions["L1"])
        front_left_foot.transform.translation.z = -self.body_dimensions["H"]
        
        back_left_foot.header.stamp = self.get_clock().now().to_msg()
        back_left_foot.header.frame_id = 'base_link'
        back_left_foot.child_frame_id = 'back_left_foot_default'
        back_left_foot.transform.translation.x = -self.body_dimensions["L"]/2
        back_left_foot.transform.translation.y = (self.body_dimensions["W"]/2+self.leg_dimensions["L1"])
        back_left_foot.transform.translation.z = -self.body_dimensions["H"]

        back_right_foot.header.stamp = self.get_clock().now().to_msg()
        back_right_foot.header.frame_id = 'base_link'
        back_right_foot.child_frame_id = 'back_right_foot_default'
        back_right_foot.transform.translation.x = -self.body_dimensions["L"]/2
        back_right_foot.transform.translation.y = -(self.body_dimensions["W"]/2+self.leg_dimensions["L1"])
        back_right_foot.transform.translation.z = -self.body_dimensions["H"]

        self.tf_broadcaster.sendTransform(front_right_foot)
        self.tf_broadcaster.sendTransform(front_left_foot)
        self.tf_broadcaster.sendTransform(back_left_foot)
        self.tf_broadcaster.sendTransform(back_right_foot)


def main():
    rclpy.init()
    node = FeetDefaultPosesFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
