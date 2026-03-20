#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class TriSteerKinematics:
    """
    三舵轮（swerve）运动学（稳定版）：
      wheel_i 位置: r_i = [R*cos(theta_i), R*sin(theta_i)]
      wheel_i 速度: v_i = [vx, vy] + wz x r_i = [vx - wz*y_i, vy + wz*x_i]
      steer_i = atan2(vy_i, vx_i)
      wheel_angular = |v_i| / wheel_radius

    稳定性措施：
      1) cmd_vel 限幅
      2) cmd_vel 超时自动刹停
      3) 舵角变化限斜率（rad/s）
      4) 轮速变化限加速度（rad/s^2）
      5) 最小转角优化（允许 steer+pi 并反转轮速，减少舵机瞬间大角度）
    """

    def __init__(self):
        # --- 底盘几何（按你下位机） ---
        self.R = rospy.get_param("~wheel_base_radius", 0.35335)  # m (353.35mm)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)  # m (你给的真实轮半径)

        # 你给的安装角：1=0°,2=120°,3=240°，且“顺时针”
        mount_deg_cw = rospy.get_param("~mount_angles_deg_cw", [180.0, -60.0, 60.0])
        # CW -> ROS(默认CCW为正): theta_ros = -theta_cw
        self.thetas = [math.radians(-d) for d in mount_deg_cw]

        # --- 指令限幅（先保守，避免炸） ---
        self.max_vx = rospy.get_param("~max_vx", 0.15)  # m/s
        self.max_vy = rospy.get_param("~max_vy", 0.15)  # m/s
        self.max_wz = rospy.get_param("~max_wz", 0.40)  # rad/s


        self.steer_sign = rospy.get_param("~steer_sign", [1, 1, 1])
        self.drive_sign = rospy.get_param("~drive_sign", [1, 1, 1])

        # --- 限斜率/限加速度（崩溃克星）---
        self.steer_rate_limit = rospy.get_param("~steer_rate_limit", 1.5)  # rad/s
        self.wheel_acc_limit  = rospy.get_param("~wheel_acc_limit", 20.0)  # rad/s^2

        # --- 超时刹停 ---
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.25)  # s

        # 发布频率
        self.rate = rospy.get_param("~rate", 50.0)

        # 上次输出（用于限斜率/限加速度）
        self.prev_steer = [0.0, 0.0, 0.0]
        self.prev_w = [0.0, 0.0, 0.0]
        self.last_t = rospy.Time.now()

        # 最近一次 cmd_vel
        self.latest = Twist()
        self.last_cmd_time = rospy.Time(0)

        # === 这里是你 Gazebo/ros_control 的控制器话题 ===
        self.pub_steer = [
            rospy.Publisher("/joint11_position_controller/command", Float64, queue_size=1),
            rospy.Publisher("/joint21_position_controller/command", Float64, queue_size=1),
            rospy.Publisher("/joint31_position_controller/command", Float64, queue_size=1),
        ]
        self.pub_drive = [
            rospy.Publisher("/joint12_velocity_controller/command", Float64, queue_size=1),
            rospy.Publisher("/joint22_velocity_controller/command", Float64, queue_size=1),
            rospy.Publisher("/joint32_velocity_controller/command", Float64, queue_size=1),
        ]

        rospy.Subscriber("cmd_vel", Twist, self.cb_cmd, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.on_timer)

        rospy.loginfo("tri_steer_kinematics started. R=%.5f wheel_r=%.5f", self.R, self.wheel_radius)

    def cb_cmd(self, msg: Twist):
        # 限幅
        self.latest.linear.x = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self.latest.linear.y = clamp(msg.linear.y, -self.max_vy, self.max_vy)
        self.latest.angular.z = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self.last_cmd_time = rospy.Time.now()

    def on_timer(self, _evt):
        # dt
        now = rospy.Time.now()
        dt = (now - self.last_t).to_sec()
        if dt <= 0.0:
            dt = 1e-3
        self.last_t = now

        # 超时刹停
        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            vx = vy = wz = 0.0
        else:
            vx = self.latest.linear.x
            vy = self.latest.linear.y
            wz = self.latest.angular.z

        # 逐轮计算并发布
        for i in range(3):
            theta = self.thetas[i]
            x_i = self.R * math.cos(theta)
            y_i = self.R * math.sin(theta)

            # 轮中心速度（底盘平移 + 旋转项）
            vxi = vx - wz * y_i
            vyi = vy + wz * x_i

            speed = math.hypot(vxi, vyi)  # m/s

            if speed < 1e-6:
                # 小速度：保持舵角不抖，驱动归零
                steer_des = self.prev_steer[i]
                w_des = 0.0
            else:
                steer_raw = wrap_to_pi(math.atan2(vyi, vxi))
                w_raw = speed / max(self.wheel_radius, 1e-6)  # rad/s

                # 最小转角优化（ steer 或 steer+pi ）
                a1 = wrap_to_pi(steer_raw)
                a2 = wrap_to_pi(steer_raw + math.pi)

                d1 = abs(wrap_to_pi(a1 - self.prev_steer[i]))
                d2 = abs(wrap_to_pi(a2 - self.prev_steer[i]))

                if d2 < d1:
                    steer_des = a2
                    w_des = -w_raw
                else:
                    steer_des = a1
                    w_des = w_raw

            # --- 舵角限斜率（防止 position 控制器瞬间打爆力矩）---
            d = wrap_to_pi(steer_des - self.prev_steer[i])
            max_d = self.steer_rate_limit * dt
            d = clamp(d, -max_d, +max_d)
            steer_out = wrap_to_pi(self.prev_steer[i] + d)

            # --- 轮速限加速度（防止速度控制器瞬间冲击）---
            dw = w_des - self.prev_w[i]
            max_dw = self.wheel_acc_limit * dt
            dw = clamp(dw, -max_dw, +max_dw)
            w_out = self.prev_w[i] + dw

            # 保存并发布
            self.prev_steer[i] = steer_out
            self.prev_w[i] = w_out

            self.pub_steer[i].publish(Float64(self.steer_sign[i] * steer_out))
            self.pub_drive[i].publish(Float64(self.drive_sign[i] * w_out))


def main():
    rospy.init_node("tri_steer_kinematics")
    TriSteerKinematics()
    rospy.spin()

if __name__ == "__main__":
    main()
