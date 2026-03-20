#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, termios, tty
import rospy
from geometry_msgs.msg import Twist

class KeyboardTeleopFixed:
    """
    W/S: x+ x-
    A/D: y+ y-
    Q/E: yaw+ yaw-
    SPACE: stop
    X: exit
    """
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel_manual", Twist, queue_size=1)
        self.v_cmd = rospy.get_param("~v_cmd", 0.2)
        self.w_cmd = rospy.get_param("~w_cmd", 0.6)
        self.rate_hz = rospy.get_param("~rate", 30.0)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return ch

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                t = Twist()

                if key in ['w','W']: t.linear.x =  self.v_cmd
                elif key in ['s','S']: t.linear.x = -self.v_cmd
                elif key in ['a','A']: t.linear.y =  self.v_cmd
                elif key in ['d','D']: t.linear.y = -self.v_cmd
                elif key in ['q','Q']: t.angular.z =  self.w_cmd
                elif key in ['e','E']: t.angular.z = -self.w_cmd
                elif key == ' ': pass
                elif key in ['x','X']: break

                self.pub.publish(t)
                rate.sleep()
        finally:
            # 退出前刹停一次
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rospy.init_node("tri_steer_keyboard_teleop", anonymous=False)
    KeyboardTeleopFixed().run()
