# Omnidirectional PID Local Planner 笔记

## 1. 节点作用
本节点实现一个最简的全向底盘 PID 路径跟随控制器。
不进行轨迹采样与避障，仅跟随上层规划器发布的 /opt_path。

## 2. 输入 / 输出

### 订阅
- /opt_path (nav_msgs/Path)
  全局或局部规划器生成的路径
- /truth_pose_odom 或 /odom_carto
  机器人当前位姿

### 发布
- /cmd_vel_auto (geometry_msgs/Twist)
  底盘速度指令
- /local_goal (visualization_msgs/Marker)
  前瞻点可视化
- /local_path (nav_msgs/Path)
  当前周期使用的局部路径段

## 3. 控制逻辑概述
1. 在路径上寻找距离当前位姿最近的点 nearest
2. 从 nearest 向前累计路径长度，找到前瞻点
3. 将世界坐标误差旋转到机器人坐标系
4. 使用 PID 计算 vx / vy / wz
5. 限幅后发布速度

## 4. 关键参数
- LOOKAHEAD_DIST：前瞻距离，决定控制平滑性
- KP_X / KP_Y / KP_YAW：位置与航向比例系数
- MAX_VX / MAX_VY / MAX_WZ：速度限幅

## 5. 已知问题 / 注意事项
- 本节点不具备避障能力
- CMD_TIMEOUT 参数当前未生效（待补充）
- 路径 frame 必须与里程计 frame 一致


