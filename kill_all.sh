#!/usr/bin/env bash
set -e

# 1) 清理旧进程（允许失败）
rosnode kill -a 2>/dev/null || true
killall -9 gzserver gzclient 2>/dev/null || true
killall -9 roscore rosmaster 2>/dev/null || true

sleep 2

# 将所有的终端都关闭
killall gnome-terminal-server


