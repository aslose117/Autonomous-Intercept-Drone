#!/bin/bash

# =======================================================
# PX4 双机仿真启动脚本 (纯净版)
# Drone 1: gz_x500 (Instance 1)
# Drone 2: gz_x500 (Instance 2)
# 世界: grass_world
# =======================================================

set -e

# -------------------------------------------------------
# 0. 检查工作目录 & 依赖工具
# -------------------------------------------------------
if [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    echo "❌ 错误：请在 PX4-Autopilot 根目录下运行此脚本"
    echo "   当前目录: $(pwd)"
    exit 1
fi

if ! command -v rotatelogs &>/dev/null; then
    echo "❌ 错误：未找到 rotatelogs，请安装 apache2-utils"
    echo "   sudo apt install apache2-utils"
    exit 1
fi

# 日志轮转配置
LOG_MAX_SIZE="5M"
LOG_MAX_FILES=2

# -------------------------------------------------------
# 1. 彻底清理残留进程
# -------------------------------------------------------
echo "🧹 正在清理残留进程..."
killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent 2>/dev/null || true
sleep 2

# 清理旧日志
rm -f ./*.log

# 清理 PX4 仿真缓存
PX4_LOG_DIR="build/px4_sitl_default/tmp/rootfs/log"
if [ -d "$PX4_LOG_DIR" ]; then
    rm -rf "${PX4_LOG_DIR:?}"/*
fi

# -------------------------------------------------------
# 2. 启动 MicroXRCEAgent (ROS 2 通信桥)
# -------------------------------------------------------
echo ""
echo "📡 正在启动 MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 2>&1 | rotatelogs -n $LOG_MAX_FILES agent.log $LOG_MAX_SIZE &
PID_AGENT=$!
echo "   -> Agent PID: $PID_AGENT"
sleep 2

# -------------------------------------------------------
# 3. 启动第一架无人机 (Instance 1)
#    此进程会同时拉起 Gazebo 仿真服务器
# -------------------------------------------------------
echo ""
echo "🚀 正在启动 Drone 1 (gz_x500, Instance 1)..."
PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,0" \
PX4_SIM_MODEL=gz_x500 \
    ./build/px4_sitl_default/bin/px4 -i 1 2>&1 | rotatelogs -n $LOG_MAX_FILES drone1.log $LOG_MAX_SIZE &
PID1=$!
echo "   -> Drone 1 PID: $PID1"

echo ""
echo "⏳ 等待 Gazebo 世界加载 (10 秒)..."
sleep 10

# -------------------------------------------------------
# 4. 启动第二架无人机 (Instance 2)
#    PX4_GZ_STANDALONE=1 表示连接已有的 Gazebo 服务器
# -------------------------------------------------------
echo ""
echo "🚀 正在启动 Drone 2 (gz_x500, Instance 2)..."
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="20,0" \
PX4_SIM_MODEL=gz_x500 \
    ./build/px4_sitl_default/bin/px4 -i 2 2>&1 | rotatelogs -n $LOG_MAX_FILES drone2.log $LOG_MAX_SIZE &
PID2=$!
echo "   -> Drone 2 PID: $PID2"
sleep 3

# -------------------------------------------------------
# 5. 启动摘要
# -------------------------------------------------------
echo ""
echo "✅ 所有系统启动完成！"
echo "================================================"
echo "  MicroXRCEAgent : PID $PID_AGENT  (port 8888)"
echo "  Drone 1 (Standard) : PID $PID1      Instance 1  (0, 0)"
echo "  Drone 2 (Standard) : PID $PID2      Instance 2  (2, 0)"
echo "------------------------------------------------"
echo "  验证命令:"
echo "  ros2 topic list | grep px4"
echo "================================================"
echo ""
echo "按 Ctrl+C 关闭所有仿真进程"

# -------------------------------------------------------
# 6. 退出时清理所有子进程
# -------------------------------------------------------
cleanup() {
    echo ""
    echo "🛑 正在关闭所有进程..."
    kill "$PID1" "$PID2" "$PID_AGENT" 2>/dev/null || true
    sleep 1
    killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent 2>/dev/null || true
    echo "✅ 清理完成"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 等待主进程退出
wait "$PID1" "$PID2" "$PID_AGENT"
