#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry


class OdomSender(Node):
    def __init__(self):
        super().__init__("odom_sender")
        self.pub = self.create_publisher(Odometry, "odom", 10)
        self.timer = self.create_timer(1.0, self.publish_odom)
        self.counter = 0
        self.msg = self._build_msg()

    def _build_msg(self):
        msg = Odometry()
        msg.header.frame_id = "world"

        q = quaternion_from_euler(0, 0, 0, "rzyx")

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        return msg

    def publish_odom(self):
        self.counter += 1
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.get_logger().info(f"Send {self.counter:3d} msg(s).")


def main():
    rclpy.init()
    node = OdomSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
