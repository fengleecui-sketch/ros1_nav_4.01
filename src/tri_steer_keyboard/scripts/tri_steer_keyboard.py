#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import termios
import tty
import rospy
from geometry_msgs.msg import Twist

class KeyboardTeleopFixed:
    """
    固定档位键盘控制（更稳，不会因为按键自动连发而速度越加越大）
      W/S: 前进/后退（x）
      A/D: 左移/右移（y）
      Q/E: 左转/右转（yaw）
      SPACE: 急停
      X: 退出
    松手：发送 0（立即停）
    """

    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.v_cmd = rospy.get_param("~v_cmd", 0.15)   # m/s
        self.w_cmd = rospy.get_param("~w_cmd", 0.40)   # rad/s
        self.rate_hz = rospy.get_param("~rate", 30.0)

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key_blocking(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def run(self):
        r = rospy.Rate(self.rate_hz)

        print("\nTri-steer keyboard teleop (fixed speed):")
        print("  W/S: +x/-x, A/D: +y/-y, Q/E: +yaw/-yaw, SPACE: stop, X: exit")
        print(f"  v_cmd={self.v_cmd:.3f} m/s, w_cmd={self.w_cmd:.3f} rad/s\n")

        # 先发一次停
        self.publish_twist(0.0, 0.0, 0.0)

        while not rospy.is_shutdown():
            key = self.get_key_blocking()

            if key in ('w', 'W'):
                self.publish_twist(+self.v_cmd, 0.0, 0.0)
            elif key in ('s', 'S'):
                self.publish_twist(-self.v_cmd, 0.0, 0.0)
            elif key in ('a', 'A'):
                self.publish_twist(0.0, self.v_cmd, 0.0) # A: 左移
            elif key in ('d', 'D'):
                self.publish_twist(0.0, -self.v_cmd, 0.0) # D: 右移
            elif key in ('q', 'Q'):
                self.publish_twist(0.0, 0.0, self.w_cmd)
            elif key in ('e', 'E'):
                self.publish_twist(0.0, 0.0, -self.w_cmd)
            elif key == ' ':
                self.publish_twist(0.0, 0.0, 0.0)
            elif key in ('x', 'X'):
                break
            else:
                # 其他键：当作松手，立即停
                self.publish_twist(0.0, 0.0, 0.0)

            r.sleep()

        # 退出前再停一次
        self.publish_twist(0.0, 0.0, 0.0)
        print("\nExit, stop published.")

def main():
    rospy.init_node("tri_steer_keyboard")
    KeyboardTeleopFixed().run()

if __name__ == "__main__":
    main()
