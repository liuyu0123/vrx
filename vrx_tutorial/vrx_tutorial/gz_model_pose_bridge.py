#!/usr/bin/env python3
"""Bridge WAM-V model pose from Gazebo /world/*/pose/info to ROS TF and /odom."""

import re
import subprocess
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class GzModelPoseBridge(Node):
    def __init__(self):
        super().__init__('gz_model_pose_bridge')

        self.declare_parameter('model_name', 'wamv')
        self.declare_parameter('world_name', '')  # auto-detect if empty
        self.declare_parameter('base_frame_id', 'wamv/wamv/base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 20.0)

        self.model_name = self.get_parameter('model_name').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        world_name = self.get_parameter('world_name').value

        if not world_name:
            world_name = self._detect_world_name()
        self.gz_topic = f'/world/{world_name}/pose/info'

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)

        self._latest_pose = None
        self._pose_lock = threading.Lock()

        # Start Gazebo topic listener in background thread
        self._gz_thread = threading.Thread(target=self._gz_listener, daemon=True)
        self._gz_thread.start()

        period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Listening to Gazebo {self.gz_topic} for model "{self.model_name}" '
            f'and publishing {self.odom_frame_id} -> {self.base_frame_id}'
        )

    def _detect_world_name(self) -> str:
        try:
            output = subprocess.check_output(
                ['gz', 'topic', '-l'], text=True, timeout=5)
            for line in output.splitlines():
                if '/world/' in line and '/pose/info' in line:
                    world = line.split('/world/')[1].split('/')[0]
                    self.get_logger().info(f'Auto-detected world name: {world}')
                    return world
        except Exception as e:
            self.get_logger().warn(f'Failed to auto-detect world name: {e}')
        return 'sydney_regatta'

    def _gz_listener(self):
        cmd = ['gz', 'topic', '-e', '-t', self.gz_topic]
        self.get_logger().info(f'Starting subprocess: {" ".join(cmd)}')
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        buffer_lines = []
        brace_depth = 0
        in_wamv_block = False

        for line in proc.stdout:
            if not rclpy.ok():
                break

            stripped = line.strip()
            if stripped == 'pose {':
                # Parse previous block if it was wamv
                if in_wamv_block and buffer_lines:
                    text = ''.join(buffer_lines)
                    pose = self._parse_wamv_pose(text)
                    if pose is not None:
                        with self._pose_lock:
                            self._latest_pose = pose
                buffer_lines = [line]
                brace_depth = 1
                in_wamv_block = False
                continue

            if brace_depth == 0:
                continue

            buffer_lines.append(line)
            brace_depth += line.count('{')
            brace_depth -= line.count('}')

            if f'name: "{self.model_name}"' in line:
                in_wamv_block = True

            if brace_depth == 0 and in_wamv_block:
                text = ''.join(buffer_lines)
                pose = self._parse_wamv_pose(text)
                if pose is not None:
                    with self._pose_lock:
                        self._latest_pose = pose
                buffer_lines = []
                in_wamv_block = False

        self.get_logger().warn('gz topic subprocess exited')

    def _parse_wamv_pose(self, text: str):
        pos_match = re.search(r'position\s*\{([^}]*)\}', text)
        ori_match = re.search(r'orientation\s*\{([^}]*)\}', text)
        if not pos_match or not ori_match:
            return None

        pos = {}
        for m in re.finditer(r'\b(x|y|z):\s*([-\d.eE]+)', pos_match.group(1)):
            pos[m.group(1)] = float(m.group(2))

        ori = {}
        for m in re.finditer(r'\b(x|y|z|w):\s*([-\d.eE]+)', ori_match.group(1)):
            ori[m.group(1)] = float(m.group(2))

        if len(pos) == 3 and len(ori) == 4:
            return pos, ori
        return None

    def _publish(self):
        with self._pose_lock:
            pose = self._latest_pose

        if pose is None:
            self.get_logger().warn(
                'No pose received from Gazebo yet', throttle_duration_sec=5.0)
            return

        (pos, ori) = pose
        now = self.get_clock().now().to_msg()

        # Publish TF: odom -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame_id
        tf_msg.child_frame_id = self.base_frame_id
        tf_msg.transform.translation.x = pos['x']
        tf_msg.transform.translation.y = pos['y']
        tf_msg.transform.translation.z = pos['z']
        tf_msg.transform.rotation.x = ori['x']
        tf_msg.transform.rotation.y = ori['y']
        tf_msg.transform.rotation.z = ori['z']
        tf_msg.transform.rotation.w = ori['w']
        self.tf_broadcaster.sendTransform(tf_msg)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = pos['x']
        odom.pose.pose.position.y = pos['y']
        odom.pose.pose.position.z = pos['z']
        odom.pose.pose.orientation.x = ori['x']
        odom.pose.pose.orientation.y = ori['y']
        odom.pose.pose.orientation.z = ori['z']
        odom.pose.pose.orientation.w = ori['w']
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GzModelPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
