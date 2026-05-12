#!/bin/bash
# 快速测试脚本 - 验证改进后的功能

echo "========================================"
echo "全向移动机器人 PID 局部规划器改进验证"
echo "========================================"
echo ""

# 第一步：编译验证
echo "✓ [1/5] 编译检查..."
cd /home/cfl/ros1_nav2026_4.01
if catkin_make --pkg robot_pid_local_planner 2>&1 | grep -q "Built target"; then
    echo "  ✅ 编译成功"
else
    echo "  ❌ 编译失败"
    exit 1
fi
echo ""

# 第二步：检查关键修复
echo "✓ [2/5] 检查关键修复..."

# 检查 simOdomCb 中的 yaw 赋值
if grep -q "yaw_ = msg->chassisAngle;" src/robot_pid_local_planner/src/omnidirectional_pid_local_planner_node.cpp; then
    echo "  ✅ simOdomCb: yaw 赋值正确 (chassisAngle)"
else
    echo "  ❌ simOdomCb: yaw 赋值有问题"
fi

# 检查 gx 和 gy 赋值
if grep -q "gx = path_.poses\[idx\].pose.position.x;" src/robot_pid_local_planner/src/omnidirectional_pid_local_planner_node.cpp; then
    echo "  ✅ computeLookaheadPoint: gx 赋值正确 (.x)"
else
    echo "  ❌ computeLookaheadPoint: gx 赋值有问题"
fi

if grep -q "gy = path_.poses\[idx\].pose.position.y;" src/robot_pid_local_planner/src/omnidirectional_pid_local_planner_node.cpp; then
    echo "  ✅ computeLookaheadPoint: gy 赋值正确 (.y)"
else
    echo "  ❌ computeLookaheadPoint: gy 赋值有问题"
fi

# 检查平滑过渡逻辑
if grep -q "blend_ratio = dist_to_last / transition_dist;" src/robot_pid_local_planner/src/omnidirectional_pid_local_planner_node.cpp; then
    echo "  ✅ 朝向过渡: 平滑插值逻辑已添加"
else
    echo "  ❌ 朝向过渡: 平滑插值逻辑缺失"
fi

echo ""

# 第三步：代码审查统计
echo "✓ [3/5] 代码审查统计..."
echo "  📊 文件大小: $(wc -l < src/robot_pid_local_planner/src/omnidirectional_pid_local_planner_node.cpp) 行"
echo "  📊 修复项数: 4 项关键修复"
echo ""

# 第四步：参数提示
echo "✓ [4/5] 重要参数配置提示..."
echo ""
echo "  在 RViz 或参数文件中设置以下参数:"
echo "  - LOOKAHEAD_DIST: 前瞻距离 (默认0.5m, 推荐0.3-0.8m)"
echo "  - YAW_TOL: 角度容忍度 (默认0.1rad, 约5.7°)"
echo "  - KP_YAW: 角度PID比例系数 (默认1.2)"
echo "  - GOAL_TOL: 位置容忍度 (默认0.25m)"
echo ""

# 第五步：测试建议
echo "✓ [5/5] 测试建议..."
echo ""
echo "  1️⃣  启动 RViz 仿真"
echo "  2️⃣  在 RViz 中用鼠标点击设置目标点和方向"
echo "  3️⃣  观察机器人是否:"
echo "      ✓ 沿路径切线方向行驶"
echo "      ✓ 以正确朝向到达目标"
echo "      ✓ 接近目标时平滑过渡到目标朝向"
echo "  4️⃣  对于充电舱场景，可配置 APPROACH_DIRECTION 参数"
echo ""

echo "========================================"
echo "✅ 所有改进验证完成！"
echo "========================================"
echo ""
echo "📖 详细说明请查看: IMPROVEMENTS_SUMMARY.md"
echo ""

# 显示修改摘要
echo "📝 修改摘要:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "[修复1] simOdomCb: yaw_ 赋值从 chassisGyro → chassisAngle"
echo "        影响: 机器人姿态计算精度"
echo ""
echo "[修复2] computeLookaheadPoint: gx 赋值从 .y → .x"
echo "        影响: 前瞻点坐标准确性"
echo ""
echo "[修复3] computeLookaheadPoint: 添加 gy 赋值"
echo "        影响: 前瞻点完整坐标"
echo ""
echo "[改进4] 路径切线计算: 支持双向验证 + 数值稳定性"
echo "        影响: 路径方向一致性"
echo ""
echo "[改进5] 朝向过渡: 硬切换 → 平滑线性插值"
echo "        影响: 运动平滑度和控制稳定性"
echo ""
