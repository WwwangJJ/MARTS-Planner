#!/usr/bin/env python3

import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster


class OdometryConverter:
    def __init__(self, node, frame_id_in, frame_id_out, broadcast_tf, body_frame_id,
                 intermediate_frame_id, world_frame_id, broadcaster):
        self.node = node
        self.frame_id_in = frame_id_in
        self.frame_id_out = frame_id_out
        self.broadcast_tf = broadcast_tf
        self.body_frame_id = body_frame_id
        self.intermediate_frame_id = intermediate_frame_id
        self.world_frame_id = world_frame_id
        self.broadcaster = broadcaster
        self.in_odom_sub = None
        self.out_odom_pub = None
        self.out_path_pub = None
        self.path_pub_timer = None
        self.tf_pub_flag = True
        if self.broadcast_tf:
            self.node.get_logger().info(
                f"ROSTopic: [{self.frame_id_in}]->[{self.frame_id_out}] TF: [{self.body_frame_id}]-[{self.intermediate_frame_id}]-[{self.world_frame_id}]"
            )
        else:
            self.node.get_logger().info(
                f"ROSTopic: [{self.frame_id_in}]->[{self.frame_id_out}] No TF"
            )

        self.path = []

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = euler_from_quaternion(q, 'rzyx')
        wqb = quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = quaternion_from_euler(e[0], 0.0, 0.0, 'rzyx')

        #### odom ####
        odom_msg = in_odom_msg
        if in_odom_msg.header.frame_id != self.frame_id_in:
            self.node.get_logger().warn("Incoming odom frame_id does not match expected frame.")
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        #### tf ####
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            if self.frame_id_in != self.frame_id_out:
                self._send_transform((0.0, 0.0, 0.0), quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                                     odom_msg.header.stamp, self.frame_id_in, self.frame_id_out)

            if self.world_frame_id != self.frame_id_out:
                self._send_transform((0.0, 0.0, 0.0), quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                                     odom_msg.header.stamp, self.world_frame_id, self.frame_id_out)

            self._send_transform((p[0], p[1], p[2]), wqb,
                                 odom_msg.header.stamp, self.body_frame_id, self.world_frame_id)

            self._send_transform((p[0], p[1], p[2]), wqc,
                                 odom_msg.header.stamp, self.intermediate_frame_id, self.world_frame_id)
        #### path ####
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.append(pose)

    def _send_transform(self, translation, quat, stamp, child_frame, parent_frame):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.broadcaster.sendTransform(t)

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self):
        self.tf_pub_flag = True


class TfAssistNode(Node):
    def __init__(self):
        super().__init__('tf_assist')
        self.converters = []
        self.broadcaster = TransformBroadcaster(self)
        self._setup_converters()

    def _setup_converters(self):
        index = 0
        while True:
            prefix = f"converter{index}."
            frame_id_in = self.declare_parameter(f"{prefix}frame_id_in", "").value
            if not frame_id_in:
                if index == 0:
                    raise RuntimeError("No converter parameters found. Set converter0.frame_id_in to start.")
                if index == 1:
                    self.get_logger().info(
                        f'prefix:"{prefix}" not found. Generate {index} converter.'
                    )
                else:
                    self.get_logger().info(
                        f'prefix:"{prefix}" not found. Generate {index} converters'
                    )
                break

            frame_id_out = self.declare_parameter(f"{prefix}frame_id_out", "").value
            broadcast_tf = self.declare_parameter(f"{prefix}broadcast_tf", False).value
            body_frame_id = self.declare_parameter(f"{prefix}body_frame_id", "body").value
            intermediate_frame_id = self.declare_parameter(f"{prefix}intermediate_frame_id", "intermediate").value
            world_frame_id = self.declare_parameter(f"{prefix}world_frame_id", "world").value

            converter = OdometryConverter(
                self, frame_id_in, frame_id_out, broadcast_tf, body_frame_id,
                intermediate_frame_id, world_frame_id, self.broadcaster
            )
            converter.in_odom_sub = self.create_subscription(
                Odometry, f"{prefix}in_odom", converter.in_odom_callback, 10
            )
            converter.out_odom_pub = self.create_publisher(
                Odometry, f"{prefix}out_odom", 10
            )
            converter.out_path_pub = self.create_publisher(
                Path, f"{prefix}out_path", 10
            )

            converter.tf_pub_timer = self.create_timer(0.1, converter.tf_pub_callback)
            converter.path_pub_timer = self.create_timer(0.5, converter.path_pub_callback)

            self.converters.append(converter)
            index += 1


def main():
    rclpy.init()
    node = TfAssistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
