#!/usr/bin/env python3
"""Convert geometry_msgs/Twist (cmd_vel) to WAM-V thruster commands."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVelToWamv(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wamv')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')
        self.declare_parameter('left_pos_topic', '/wamv/thrusters/left/pos')
        self.declare_parameter('right_pos_topic', '/wamv/thrusters/right/pos')
        self.declare_parameter('max_thrust', 1000.0)
        self.declare_parameter('max_pos', 1.0)
        self.declare_parameter('thrust_scale', 500.0)
        self.declare_parameter('pos_scale', 0.5)

        self.max_thrust = self.get_parameter('max_thrust').value
        self.max_pos = self.get_parameter('max_pos').value
        self.thrust_scale = self.get_parameter('thrust_scale').value
        self.pos_scale = self.get_parameter('pos_scale').value

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        left_topic = self.get_parameter('left_thrust_topic').value
        right_topic = self.get_parameter('right_thrust_topic').value
        left_pos_topic = self.get_parameter('left_pos_topic').value
        right_pos_topic = self.get_parameter('right_pos_topic').value

        self.cmd_sub = self.create_subscription(Twist, cmd_topic, self.cmd_callback, 10)

        self.left_pub = self.create_publisher(Float64, left_topic, 10)
        self.right_pub = self.create_publisher(Float64, right_topic, 10)
        self.left_pos_pub = self.create_publisher(Float64, left_pos_topic, 10)
        self.right_pos_pub = self.create_publisher(Float64, right_pos_topic, 10)

        self.last_cmd_time = self.get_clock().now()
        self.timeout_sec = 0.5

        # Watchdog timer: if no cmd_vel received, stop thrusters
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info('CmdVelToWamv node started.')
        self.get_logger().info(
            f'Params: max_thrust={self.max_thrust}, max_pos={self.max_pos}, '
            f'thrust_scale={self.thrust_scale}, pos_scale={self.pos_scale}'
        )

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Compute thrust from linear velocity
        thrust = linear * self.thrust_scale
        thrust = max(-self.max_thrust, min(self.max_thrust, thrust))

        # Compute rudder angle from angular velocity
        # Negative angular.z -> turn left -> positive rudder angle (point right)
        pos = -angular * self.pos_scale
        pos = max(-self.max_pos, min(self.max_pos, pos))

        self.left_pub.publish(Float64(data=float(thrust)))
        self.right_pub.publish(Float64(data=float(thrust)))
        self.left_pos_pub.publish(Float64(data=float(pos)))
        self.right_pos_pub.publish(Float64(data=float(pos)))

    def watchdog(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            self.left_pub.publish(Float64(data=0.0))
            self.right_pub.publish(Float64(data=0.0))
            self.left_pos_pub.publish(Float64(data=0.0))
            self.right_pos_pub.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWamv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left_pub.publish(Float64(data=0.0))
        node.right_pub.publish(Float64(data=0.0))
        node.left_pos_pub.publish(Float64(data=0.0))
        node.right_pos_pub.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
