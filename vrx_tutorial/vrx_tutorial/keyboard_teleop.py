#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import select
import termios
import tty


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('max_thrust', 1000.0)
        self.declare_parameter('thrust_step', 50.0)
        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')
        self.declare_parameter('left_pos_topic', '/wamv/thrusters/left/pos')
        self.declare_parameter('right_pos_topic', '/wamv/thrusters/right/pos')

        self.max_thrust = self.get_parameter('max_thrust').value
        self.thrust_step = self.get_parameter('thrust_step').value

        left_topic = self.get_parameter('left_thrust_topic').value
        right_topic = self.get_parameter('right_thrust_topic').value
        left_pos_topic = self.get_parameter('left_pos_topic').value
        right_pos_topic = self.get_parameter('right_pos_topic').value

        self.left_pub = self.create_publisher(Float64, left_topic, 10)
        self.right_pub = self.create_publisher(Float64, right_topic, 10)
        self.left_pos_pub = self.create_publisher(Float64, left_pos_topic, 10)
        self.right_pos_pub = self.create_publisher(Float64, right_pos_topic, 10)

        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0

        # 保存终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.print_help()

    def print_help(self):
        print("""
========================================
        VRX 键盘遥控
========================================
    W / ↑    : 增加推力（前进）
    S / ↓    : 减少推力（后退）
    A / ←    : 差速左转
    D / →    : 差速右转
c / Space    : 停止（推力归零）
    Q        : 退出
========================================
""")

    def get_key(self):
        """非阻塞读取一个按键（Linux only）"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def control_loop(self):
        key = self.get_key()

        if key is not None:
            key_lower = key.lower()

            if key_lower == 'w' or key == '\x1b[A':  # W 或 上箭头
                self.left_thrust += self.thrust_step
                self.right_thrust += self.thrust_step
                print(f"[前进] left={self.left_thrust:.0f} right={self.right_thrust:.0f}")

            elif key_lower == 's' or key == '\x1b[B':  # S 或 下箭头
                self.left_thrust -= self.thrust_step
                self.right_thrust -= self.thrust_step
                print(f"[后退] left={self.left_thrust:.0f} right={self.right_thrust:.0f}")

            elif key_lower == 'a' or key == '\x1b[D':  # A 或 左箭头
                self.left_thrust -= self.thrust_step
                self.right_thrust += self.thrust_step
                print(f"[左转] left={self.left_thrust:.0f} right={self.right_thrust:.0f}")

            elif key_lower == 'd' or key == '\x1b[C':  # D 或 右箭头
                self.left_thrust += self.thrust_step
                self.right_thrust -= self.thrust_step
                print(f"[右转] left={self.left_thrust:.0f} right={self.right_thrust:.0f}")

            elif key_lower == 'c' or key == ' ':
                self.left_thrust = 0.0
                self.right_thrust = 0.0
                print("[停止]")

            elif key_lower == 'q':
                print("[退出]")
                self.stop_and_shutdown()
                return

            # 推力限幅
            self.left_thrust = max(-self.max_thrust, min(self.max_thrust, self.left_thrust))
            self.right_thrust = max(-self.max_thrust, min(self.max_thrust, self.right_thrust))

        # 发布指令
        self.left_pub.publish(Float64(data=float(self.left_thrust)))
        self.right_pub.publish(Float64(data=float(self.right_thrust)))
        self.left_pos_pub.publish(Float64(data=float(self.left_pos)))
        self.right_pos_pub.publish(Float64(data=float(self.right_pos)))

    def stop_and_shutdown(self):
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.left_pub.publish(Float64(data=0.0))
        self.right_pub.publish(Float64(data=0.0))
        self.left_pos_pub.publish(Float64(data=0.0))
        self.right_pos_pub.publish(Float64(data=0.0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.destroy_node()
        rclpy.shutdown()

    def __del__(self):
        # 恢复终端设置
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_shutdown()


if __name__ == '__main__':
    main()
