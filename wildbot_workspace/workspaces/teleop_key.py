#!/usr/bin/env python3
"""
鍵盤遙控腳本
  ↑ / W   前進
  ↓ / S   後退
  ← / A   左轉
  → / D   右轉
  空格     停止
  Q        離開並停車

用法（在 wildbot 容器內）：
  source /opt/ros/jazzy/setup.bash
  source /workspaces/install/setup.bash
  python3 /workspaces/teleop_key.py
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

LINEAR_SPEED  = 0.30   # m/s
ANGULAR_SPEED = 0.40   # rad/s
PUBLISH_HZ    = 20     # 每秒發布次數


def get_key(settings):
    """讀取單次按鍵，支援方向鍵（escape sequence）。"""
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        ch2 = sys.stdin.read(1)
        ch3 = sys.stdin.read(1)
        ch = ch + ch2 + ch3
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_key")
        self.pub = self.create_publisher(
            TwistStamped, "/base_controller/cmd_vel", 10
        )
        self._linear  = 0.0
        self._angular = 0.0
        self._lock    = threading.Lock()
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def set_cmd(self, linear: float, angular: float):
        with self._lock:
            self._linear  = linear
            self._angular = angular

    def _publish(self):
        with self._lock:
            lin = self._linear
            ang = self._angular
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x  = lin
        msg.twist.angular.z = ang
        self.pub.publish(msg)

    def stop(self):
        self.set_cmd(0.0, 0.0)
        self._publish()


KEY_MAP = {
    "\x1b[A": ( LINEAR_SPEED,  0.0),           # ↑
    "\x1b[B": (-LINEAR_SPEED,  0.0),           # ↓
    "\x1b[D": ( 0.0,           ANGULAR_SPEED), # ←
    "\x1b[C": ( 0.0,          -ANGULAR_SPEED), # →
    "w":      ( LINEAR_SPEED,  0.0),
    "s":      (-LINEAR_SPEED,  0.0),
    "a":      ( 0.0,           ANGULAR_SPEED),
    "d":      ( 0.0,          -ANGULAR_SPEED),
    " ":      ( 0.0,           0.0),            # 空格停止
}

HELP = """
┌─────────────────────────────┐
│   ↑/W  前進   ↓/S  後退    │
│   ←/A  左轉   →/D  右轉    │
│   空格  停止    Q   離開    │
└─────────────────────────────┘
"""


def main():
    rclpy.init()
    node = TeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        while True:
            key = get_key(settings)
            if key.lower() == "q":
                break
            if key in KEY_MAP:
                lin, ang = KEY_MAP[key]
                node.set_cmd(lin, ang)
                action = (
                    "前進" if lin > 0 else
                    "後退" if lin < 0 else
                    "左轉" if ang > 0 else
                    "右轉" if ang < 0 else
                    "停止"
                )
                print(f"\r{action}  (linear={lin:.2f} angular={ang:.2f})    ", end="", flush=True)
    except Exception as e:
        print(f"\n錯誤: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n停車中...")
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
