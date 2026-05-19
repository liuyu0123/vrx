#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # 参数
        self.declare_parameter('safety_distance', 20.0)
        self.declare_parameter('cruise_thrust', 500.0)
        self.declare_parameter('turn_thrust', 300.0)
        self.declare_parameter('scan_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/scan')
        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')

        self.safety_distance = self.get_parameter('safety_distance').value
        self.cruise_thrust = self.get_parameter('cruise_thrust').value
        self.turn_thrust = self.get_parameter('turn_thrust').value

        scan_topic = self.get_parameter('scan_topic').value
        left_topic = self.get_parameter('left_thrust_topic').value
        right_topic = self.get_parameter('right_thrust_topic').value

        # 订阅激光雷达
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # 发布推力
        self.left_pub = self.create_publisher(Float64, left_topic, 10)
        self.right_pub = self.create_publisher(Float64, right_topic, 10)

        # 控制循环 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.latest_scan = None
        self.scan_info_printed = False

        self.get_logger().info('ObstacleAvoidance node started.')
        self.get_logger().info(f'Safety distance: {self.safety_distance} m')

    def scan_callback(self, msg: LaserScan):
        if not self.scan_info_printed:
            self.get_logger().info(
                f'LaserScan received: angle_min={msg.angle_min:.2f}, '
                f'angle_max={msg.angle_max:.2f}, angle_increment={msg.angle_increment:.4f}, '
                f'ranges_len={len(msg.ranges)}'
            )
            self.scan_info_printed = True
        self.latest_scan = msg

    def control_loop(self):
        if self.latest_scan is None:
            self.get_logger().warn('Waiting for LaserScan...', throttle_duration_sec=5.0)
            return

        msg = self.latest_scan
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        # 收集有效距离（排除 inf / nan）
        valid = [(i, r) for i, r in enumerate(ranges) if math.isfinite(r)]
        if not valid:
            self.get_logger().warn('All ranges are invalid!', throttle_duration_sec=5.0)
            return

        # 找到最小距离及其索引
        min_idx, min_dist = min(valid, key=lambda x: x[1])

        # 决策：直行 or 转弯
        # 把 ranges 分成左右两半，根据障碍在哪一侧决定转向
        mid = n // 2

        if min_dist < self.safety_distance:
            # 有障碍，转向空旷的一侧
            # 如果障碍在左半边（索引 < mid），往右转
            # 如果障碍在右半边（索引 >= mid），往左转
            if min_idx < mid:
                left_cmd = self.turn_thrust
                right_cmd = -self.turn_thrust
                action = 'turn right (obstacle on left/front-left)'
            else:
                left_cmd = -self.turn_thrust
                right_cmd = self.turn_thrust
                action = 'turn left (obstacle on right/front-right)'
        else:
            # 安全，直行
            left_cmd = self.cruise_thrust
            right_cmd = self.cruise_thrust
            action = 'cruise straight'

        # 发布指令
        self.left_pub.publish(Float64(data=float(left_cmd)))
        self.right_pub.publish(Float64(data=float(right_cmd)))

        self.get_logger().info(
            f'{action} | min_dist={min_dist:.1f}m @ idx={min_idx}/{n} | '
            f'left={left_cmd:.0f} right={right_cmd:.0f}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止推进器
        node.left_pub.publish(Float64(data=0.0))
        node.right_pub.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
