# Autonomous Intercept Drone
### 基于视觉伺服的自主拦截无人机系统 (Autonomous Intercept Drone with Image-based Visual Servo)

<div align="center">

[![PX4](https://img.shields.io/badge/PX4-Autopilot-blue.svg)](https://px4.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FFoxy-green.svg)](https://docs.ros.org/en/humble/)
[![TensorRT](https://img.shields.io/badge/TensorRT-Accelerated-76B900.svg)](https://developer.nvidia.com/tensorrt)
[![YOLO11](https://img.shields.io/badge/YOLO-v11-yellow.svg)](https://github.com/ultralytics/ultralytics)

</div>

**Autonomous Intercept Drone** 是一个基于 **Image-based Visual Servo (IBVS)** 技术的自主拦截无人机项目。本项目结合了先进的小目标检测算法与 PX4/Gazebo 仿真环境，实现了对空中移动目标的自主识别、追踪与拦截。

---

## 🏗️ 系统架构 (System Architecture)

### 1. 小目标无人机检测框架
针对远距离、弱小无人机目标的检测优化方案，确保在复杂背景下也能精准识别目标。

<div align="center">
  <img src="./assets/小目标无人机检测框架.jpg" alt="小目标无人机检测方案" width="80%">
</div>

### 2. 无人机拦截方案框架
基于视觉反馈的闭环控制策略，包含状态估计、目标预测与伺服控制逻辑。

<div align="center">
  <img src="./assets/框架.png" alt="无人机拦截框架" width="80%">
</div>

---

## 📺 仿真演示 (Simulation & Demos)

我们提供了完整的仿真视频，展示了无人机从搜索、锁定到拦截的全过程。

<div align="center">

[![Bilibili](https://img.shields.io/badge/Bilibili-观看视频演示-ff69b4?style=for-the-badge&logo=bilibili)](https://www.bilibili.com/video/BV1M8QVYHE39/?spm_id_from=333.1387.homepage.video_card.click)

<div align="center">
  <img src="./assets/output.gif" alt="演示视频" width="100%">
</div>

</div>

---

## 🛠️ 如何启动仿真环境和程序 (Simulation Environment)


本项目依赖 **PX4-Autopilot** 与 **Gazebo Sim8**。在运行以下命令前，请确保已正确配置 PX4 开发环境已搭建完成

### 1.启动Gazebo仿真环境
 在 PX4-Autopilot 根目录下运行run_swarm.sh，其中run_swarm.sh文件见仓库,(另外脚本中加载的grass_wrold见assets/gazebo_world)
```bash
cd PX4-Autopilot/
./run_swarm.sh 
```
### 2.启动无人机程序
```bash
# 在 PX4-Autopilot 根目录下运行run_swarm.sh，其中run_swarm.sh文件见仓库
ros2 run uav_target_sim  uav_target_sim
ros2 run uav_ibvs_conteol uav_ibvs_control
ros2 run uav_vision_detect uav_vision_detect
```