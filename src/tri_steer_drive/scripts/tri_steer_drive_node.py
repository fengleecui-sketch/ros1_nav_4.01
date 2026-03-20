#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class TriSteerDriveNode:
    def __init__(self):
        rospy.init_node("tri_steer_drive")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

        # Robot geometry
        self.R = float(rospy.get_param("~R", 0.2))                  # center -> wheel module distance
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.05))

        # Wheel mount angles in degrees (often provided as CW: 0/120/240)
        # Convert CW(deg) -> ROS CCW(rad): theta = -deg
        w1_deg = float(rospy.get_param("~wheel1_deg", 0.0))
        w2_deg = float(rospy.get_param("~wheel2_deg", 120.0))
        w3_deg = float(rospy.get_param("~wheel3_deg", 240.0))
        self.thetas = [
            math.radians(-w1_deg),
            math.radians(-w2_deg),
            math.radians(-w3_deg),
        ]

        # Inversion flags (fix base direction / sign conventions)
        self.invert_x = bool(rospy.get_param("~invert_x", False))
        self.invert_y = bool(rospy.get_param("~invert_y", False))
        self.invert_wz = bool(rospy.get_param("~invert_wz", False))

        # Control rate
        self.hz = float(rospy.get_param("~HZ", 50.0))

        # 180° flip mitigation (core)
        self.steer_hold_speed = float(rospy.get_param("~steer_hold_speed", 0.08))  # m/s
        self.steer_rate_limit = float(rospy.get_param("~steer_rate_limit", 2.5))   # rad/s
        self.steer_deadband = float(rospy.get_param("~steer_deadband", 0.04))       # rad (optional)
        # --- Scheme 1: flip penalty / hysteresis ---
        # Only switch to (steer+pi, -w) if it is BETTER by at least this penalty.
        self.flip_penalty_rad = float(rospy.get_param("~flip_penalty_rad", 0.6))   # rad

        # Optional limits
        self.max_wheel_w = float(rospy.get_param("~max_wheel_w", 999.0))           # rad/s
        self.max_cmd_v = float(rospy.get_param("~max_cmd_v", 999.0))               # m/s
        self.max_cmd_w = float(rospy.get_param("~max_cmd_w", 999.0))               # rad/s

        # Joint command topics (Float64)
        self.joint_steer_topics = [
            rospy.get_param("~joint_steer_1", "/joint11_position_controller/command"),
            rospy.get_param("~joint_steer_2", "/joint21_position_controller/command"),
            rospy.get_param("~joint_steer_3", "/joint31_position_controller/command"),
        ]
        self.joint_drive_topics = [
            rospy.get_param("~joint_drive_1", "/joint12_velocity_controller/command"),
            rospy.get_param("~joint_drive_2", "/joint22_velocity_controller/command"),
            rospy.get_param("~joint_drive_3", "/joint32_velocity_controller/command"),
        ]

        self.pub_steer = [rospy.Publisher(t, Float64, queue_size=10) for t in self.joint_steer_topics]
        self.pub_drive = [rospy.Publisher(t, Float64, queue_size=10) for t in self.joint_drive_topics]

        # -----------------------------
        # State
        # -----------------------------
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = rospy.Time(0)

        # steer memory to avoid pi flips
        self.last_steer = [0.0, 0.0, 0.0]
        self.last_pub_time = rospy.Time.now()

        # Subscriber
        self.sub = rospy.Subscriber(self.cmd_topic, Twist, self.cmd_cb, queue_size=10)

        rospy.loginfo("[tri_steer_drive] started. cmd_topic=%s HZ=%.1f R=%.3f wheel_r=%.3f",
                      self.cmd_topic, self.hz, self.R, self.wheel_radius)
        rospy.loginfo("[tri_steer_drive] wheel_deg(CW)=[%.1f, %.1f, %.1f] -> theta(ROS CCW rad)=[%.3f, %.3f, %.3f]",
                      w1_deg, w2_deg, w3_deg, self.thetas[0], self.thetas[1], self.thetas[2])
        rospy.loginfo("[tri_steer_drive] invert_x=%s invert_y=%s invert_wz=%s",
                      self.invert_x, self.invert_y, self.invert_wz)
        rospy.loginfo("[tri_steer_drive] steer_hold_speed=%.3f steer_rate_limit=%.2f flip_penalty_rad=%.2f deadband=%.3f",
                      self.steer_hold_speed, self.steer_rate_limit, self.flip_penalty_rad, self.steer_deadband)

    def cmd_cb(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        vx = clamp(vx, -self.max_cmd_v, self.max_cmd_v)
        vy = clamp(vy, -self.max_cmd_v, self.max_cmd_v)
        wz = clamp(wz, -self.max_cmd_w, self.max_cmd_w)

        if self.invert_x:
            vx = -vx
        if self.invert_y:
            vy = -vy
        if self.invert_wz:
            wz = -wz

        self.vx, self.vy, self.wz = vx, vy, wz
        self.last_cmd_time = rospy.Time.now()

    def wheel_cmd(self, idx: int, theta: float, dt: float):
        """
        Compute desired steer angle and wheel angular speed for wheel idx at mount angle theta.

        Includes:
          - low-speed steer hold
          - dual-solution selection with flip penalty (scheme 1)
          - steer deadband (optional)
          - steer rate limit
        """
        # wheel position in base frame
        x = self.R * math.cos(theta)
        y = self.R * math.sin(theta)

        # v_i = v + w x r = [vx - wz*y, vy + wz*x]
        vix = self.vx - self.wz * y
        viy = self.vy + self.wz * x

        speed = math.sqrt(vix * vix + viy * viy)
        prev = self.last_steer[idx]

        # 1) low-speed hold to avoid atan2 noise flips near zero
        if speed < self.steer_hold_speed:
            return prev, 0.0

        # 2) two equivalent solutions
        steer1 = wrap_to_pi(math.atan2(viy, vix))
        w1 = speed / max(1e-6, self.wheel_radius)

        steer2 = wrap_to_pi(steer1 + math.pi)
        w2 = -w1

        # 3) choose solution closest to previous steer, BUT:
        #    switching to solution2 has an extra penalty (hysteresis), reducing flip frequency.
        d1 = abs(wrap_to_pi(steer1 - prev))
        d2 = abs(wrap_to_pi(steer2 - prev))

        # Scheme 1: flip penalty
        if (d2 + self.flip_penalty_rad) < d1:
            steer = steer2
            wheel_w = w2
        else:
            steer = steer1
            wheel_w = w1

        # optional deadband (reduce tiny oscillations)
        if self.steer_deadband > 0.0:
            if abs(wrap_to_pi(steer - prev)) < self.steer_deadband:
                steer = prev

        # 4) steer rate limit
        dt = max(1e-3, dt)
        max_delta = self.steer_rate_limit * dt
        delta = wrap_to_pi(steer - prev)
        delta = clamp(delta, -max_delta, max_delta)
        steer = wrap_to_pi(prev + delta)

        # 5) clamp wheel speed if needed
        wheel_w = clamp(wheel_w, -self.max_wheel_w, self.max_wheel_w)

        self.last_steer[idx] = steer
        return steer, wheel_w

    def publish_cmds(self, steers, wheels):
        for i in range(3):
            self.pub_steer[i].publish(Float64(steers[i]))
            self.pub_drive[i].publish(Float64(wheels[i]))

    def spin(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_pub_time).to_sec()
            self.last_pub_time = now

            steers = [0.0, 0.0, 0.0]
            wheels = [0.0, 0.0, 0.0]

            for i in range(3):
                steers[i], wheels[i] = self.wheel_cmd(i, self.thetas[i], dt)

            self.publish_cmds(steers, wheels)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = TriSteerDriveNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
