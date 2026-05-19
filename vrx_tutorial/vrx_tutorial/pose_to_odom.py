#!/usr/bin/env python3
"""Publish /odom from Gazebo ground truth TF and broadcast static odom->world TF."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import StaticTransformBroadcaster


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')

        self.declare_parameter('source_frame', 'world')
        self.declare_parameter('base_frame_id', 'wamv/wamv/base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 20.0)

        self.source_frame = self.get_parameter('source_frame').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)

        # Static transform odom -> world (identity).
        # Gazebo already publishes world -> wamv/wamv/base_link via /tf,
        # so connecting odom -> world gives us map->odom->world->base_link.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = self.odom_frame_id
        static_tf.child_frame_id = self.source_frame
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_tf)

        period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self.timer_callback)

        self.last_time = None
        self.last_pos = None

        self.get_logger().info(
            f'Publishing /odom ({self.odom_frame_id} -> {self.base_frame_id}) '
            f'from {self.source_frame} -> {self.base_frame_id} TF'
        )

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame, self.base_frame_id, rclpy.time.Time())
            now = trans.header.stamp
            now_ros = rclpy.time.Time.from_msg(now)

            # Compute twist by finite difference
            twist = Odometry().twist.twist
            if self.last_time is not None and self.last_pos is not None:
                dt = (now_ros - self.last_time).nanoseconds / 1e9
                if dt > 0.0:
                    dx = trans.transform.translation.x - self.last_pos[0]
                    dy = trans.transform.translation.y - self.last_pos[1]
                    dz = trans.transform.translation.z - self.last_pos[2]
                    twist.linear.x = dx / dt
                    twist.linear.y = dy / dt
                    twist.linear.z = dz / dt

            self.last_time = now_ros
            self.last_pos = (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.child_frame_id = self.base_frame_id
            odom.pose.pose.position = trans.transform.translation
            odom.pose.pose.orientation = trans.transform.rotation
            odom.twist.twist = twist
            self.odom_pub.publish(odom)

        except tf2_ros.LookupException:
            self.get_logger().warn(
                f'TF {self.source_frame}->{self.base_frame_id} not available yet',
                throttle_duration_sec=5.0)
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                f'TF extrapolation error: {e}',
                throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(
                f'Unexpected error: {e}',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
