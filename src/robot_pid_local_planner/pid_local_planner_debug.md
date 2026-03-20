## 问题：机器人速度很慢
现象：
- local_goal 正常前移
- has_path_ = true
- vx 输出很小

原因：
- LOOKAHEAD_DIST 设为 0.3m
- 路径点密集，误差 ex 始终很小

解决：
- 将 LOOKAHEAD_DIST 调至 0.8
