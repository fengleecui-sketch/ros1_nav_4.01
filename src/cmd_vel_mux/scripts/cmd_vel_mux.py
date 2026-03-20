#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

class CmdVelMux:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.manual_timeout = rospy.get_param("~manual_timeout", 0.35)  # 手动消息多长时间内有效
        self.rate_hz = rospy.get_param("~rate", 50.0)

        self.last_manual = rospy.Time(0)
        self.last_auto = rospy.Time(0)
        self.manual = Twist()
        self.auto = Twist()

        rospy.Subscriber("/cmd_vel_manual", Twist, self.cb_manual, queue_size=1)
        rospy.Subscriber("/cmd_vel_auto", Twist, self.cb_auto, queue_size=1)

        rospy.loginfo("cmd_vel_mux started: manual=/cmd_vel_manual auto=/cmd_vel_auto out=/cmd_vel")

    def cb_manual(self, msg):
        self.manual = msg
        self.last_manual = rospy.Time.now()

    def cb_auto(self, msg):
        self.auto = msg
        self.last_auto = rospy.Time.now()

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            use_manual = (now - self.last_manual).to_sec() <= self.manual_timeout

            if use_manual:
                self.pub.publish(self.manual)
            else:
                self.pub.publish(self.auto)

            r.sleep()

if __name__ == "__main__":
    rospy.init_node("cmd_vel_mux", anonymous=False)
    CmdVelMux().spin()
