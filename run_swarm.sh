#!/bin/bash

# =======================================================
# PX4 双机仿真启动脚本 (带 ROS 2 相机桥接)
# =======================================================

# 1. 彻底清理环境
echo "🧹 正在清理残留进程..."
# 增加了 parameter_bridge 到清理列表，防止上次运行的桥接没关掉
killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent ruby parameter_bridge 2>/dev/null
sleep 2


export PX4_GZ_WORLD=grass_world

# 2. 启动通信代理 (Agent)
echo "📡 正在启动 MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 > agent.log 2>&1 &
echo "   -> Agent 已在后台运行 (日志: agent.log)"
sleep 2

# 3. 启动第一架无人机 (带相机) [Instance 0]
echo "🚀 正在启动 Drone 1 (带相机, ID:0)..."
echo "   -> 正在启动 Gazebo 界面..."
echo "   -> 加载世界: $CUSTOM_WORLD_DIR/$WORLD_NAME.sdf"

# 使用 make 启动第一架
make px4_sitl gz_x500_depth > drone1.log 2>&1 &
PID1=$!

# 等待 Gazebo 完全启动 (这一步很关键)
echo "⏳ 等待 15 秒让 Gazebo 世界加载..."
sleep 15

# =======================================================
# [新增] 启动 ROS 2 图像桥接 (Camera Bridge)
# =======================================================
echo "🌉 正在启动 ROS 2 相机桥接..."
# 将 Gazebo 内部话题 /world/.../image 转换为 ROS 2 的 /camera/image (为了方便使用，我做了重映射)
# 如果你想要原始长名字，去掉 --ros-args 及其后面的内容即可
ros2 run ros_gz_bridge parameter_bridge \
    /world/grass_world/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
    --ros-args -r /world/grass_world/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/image \
    > bridge.log 2>&1 &
PID_BRIDGE=$!
echo "   -> 桥接已启动！ROS2话题: /camera/image"

# 4. 启动第二架无人机 (普通版) [Instance 1]
echo "🚀 正在启动 Drone 2 (普通版, ID:1)..."

# 设置环境变量
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_MODEL_POSE="20,0"
# export HEADLESS=1

# 启动命令
./build/px4_sitl_default/bin/px4 -i 1 > drone2.log 2>&1 &
PID2=$!

echo "✅ 所有系统启动完成！"
echo "------------------------------------------------"
echo "Drone 1 (Camera): Instance 0"
echo "Drone 2 (Target): Instance 1"
echo "Camera Topic    : /camera/image (已重映射)"
echo "------------------------------------------------"
echo "按 Ctrl+C 关闭所有仿真"

# 等待用户退出
wait $PID1 $PID2 $PID_BRIDGE
```csv
