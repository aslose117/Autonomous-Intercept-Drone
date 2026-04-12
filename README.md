# Autonomous Intercept Drone
### 基于视觉伺服的自主拦截无人机系统 (Autonomous Intercept Drone with Image-based Visual Servo)

<div align="center">

[![Bilibili](https://img.shields.io/badge/Bilibili-观看视频演示-ff69b4?style=for-the-badge&logo=bilibili)](https://www.bilibili.com/video/BV1M8QVYHE39/?spm_id_from=333.1387.homepage.video_card.click)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-blue.svg)](https://px4.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FFoxy-green.svg)](https://docs.ros.org/en/humble/)
[![TensorRT](https://img.shields.io/badge/TensorRT-Accelerated-76B900.svg)](https://developer.nvidia.com/tensorrt)
[![YOLO11](https://img.shields.io/badge/YOLO-v11-yellow.svg)](https://github.com/ultralytics/ultralytics)

</div>

> **产品定位**：本系统是一个面向高动态目标拦截的自主无人机（UAV）实验平台。它深度集成了视觉感知、目标锁定与最优制导策略，为研究人员和开发者提供了一套从仿真到部署的完整闭环方案。

---

## 💎 核心价值主张 (Core Value)

在复杂的低空空域任务中，对非法或非合作无人机的拦截是当前安全领域的巨大挑战。本项目通过以下核心技术解决了这一难题：

*   **全链路闭环感知**：集成 YOLO 深度学习模型，针对“小目标”无人机进行专项优化，实现远距离稳定截获。
*   **双重引导机制**：结合 **PNG (比例导引)** 算法确保拦截路径的最优性（前置截击）。
*   **高保真仿真环境**：基于 Gazebo 与 PX4 软件在环（SITL），模拟真实物理特性与动态干扰。
*   **数据驱动的评估**：内置自动化绘图脚本，每一场拦截实验均有完整的运动学与视觉指标分析。

---

## 🎬 仿真演示 (Simulation & Demos)

### 1. 动态拦截全过程 (Real-time Intercept)
<div align="center">
  <img src="assets/output.gif" alt="拦截动态演示" width="80%">
  <p><i>（通过 PNG 算法预判目标航迹，实现精准的物理碰撞拦截）</i></p>
  <p><b>📺 <a href="https://www.bilibili.com/video/BV1M8QVYHE39/">点击此处前往 Bilibili 观看完整高清演示视频</a></b></p>
</div>

### 2. 小目标无人机检测框架 (Vision Detection)
<div align="center">
  <img src="assets/小目标无人机检测框架.jpg" alt="小目标检测框架" width="80%">
  <p><i>（针对无人机“点目标”优化的视觉处理流程）</i></p>
</div>

---

## 📐 系统架构与逻辑 (Architecture)

系统采用 ROS 2 分层架构，确保了极高的灵活性与实时性：

<div align="center">
  <img src="assets/框架.png" alt="框架图" width="80%">
</div>

| 模块名称 | 功能描述 | 关键包 | 状态/备注 |
| :--- | :--- | :--- | :--- |
| **感知层** | 目标检测、像素误差计算、视线角提取 | `uav_vision_dectect`, `uav_vision_png` | **当前主方案** |
| **制导层** | PNG 导引率计算、前置量补、轨迹生成 | `uav_png_intercept` | **纯PNG方案 (不依赖视觉)** |
| **控制层** | PX4 Offboard 接口、速度/姿态闭环控制 | `uav_vehicle_controller`, `px4_ros_com` | 核心控制底座 |
| **仿真层** | 目标机动态模拟、拦截环境生成 | `uav_target_sim` | 仿真支持 |
| **早期方案** | 基于图像的视觉伺服控制 | `uav_ibvs_control` | **已放弃 (代码仅供保留参考)** |

---

## 📊 拦截性能量化分析 (Analytics)

我们不只展示效果，更注重数据的严谨性。以下是一次典型成功拦截任务的性能报告：

### 运动学性能 (Kinematics)
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/1_3D_Trajectory.png" width="300px"><br><b>3D 拦截轨迹图</b></td>
      <td><img src="assets/plots_output/2_Relative_Distance.png" width="300px"><br><b>相对距离收敛曲线</b></td>
      <td><img src="assets/plots_output/3_Velocity.png" width="300px"><br><b>拦截机速度矢量</b></td>
    </tr>
    <tr>
      <td>展示了 3D 空间内的截击前置量</td>
      <td>验证拦截距离最终收敛至 &lt;0.2m</td>
      <td>反映了制导律对推力的高效利用</td>
    </tr>
  </table>
</div>

### 视觉追踪表现 (Vision Performance)
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/4_LOS_PNG_Angles.png" width="300px"><br><b>视线角 (LOS) 演变</b></td>
      <td><img src="assets/plots_output/5_Pixel_Error.png" width="300px"><br><b>视觉中心追踪误差</b></td>
      <td><img src="assets/plots_output/6_2D_Top_View.png" width="300px"><br><b>2D 俯视截击路径</b></td>
    </tr>
    <tr>
      <td>导引规律的收敛稳定性分析</td>
      <td>检测算法在动态过程中的稳健性表现</td>
      <td>典型前置量补效果展示</td>
    </tr>
  </table>
</div>

---

## 🛠 快速上手 (Quick Start)

### 1. 环境依赖
*   **ROS 2 Version**: Humble
*   **PX4 Firmware**: v1.16
*   **Dependencies**: OpenCV, PyTorch, colcon

### 2. 编译项目
```bash
# 进入工作空间并编译
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 运行拦截任务 (步骤序列)
请按以下顺序在不同终端中运行各节点：

1. **启动目标机仿真**:
   ```bash
   ros2 run uav_target_sim uav_target_sim
   ```
2. **启动视觉检测**:
   ```bash
   ros2 run uav_vision_dectect uav_vision_dectect
   ```
3. **启动视觉制导拦截**:
   ```bash
   ros2 run uav_vision_png uav_vision_png
   ```

---

## 📈 演进路线 (Roadmap)
- [x] 基于视觉伺服的稳定追踪 (Early Phase)
- [x] 3D 比例导引拦截算法实现 (Pure PNG & Vision-based)
- [ ] 集群协同拦截（Swarm Interception）
- [ ] 针对不确定性目标的预测控制（MPC）

---

> **项目维护者注**：如果您需要更多关于算法推导（PNG）的细节，请参考各功能包下的 `include/` 头文件或联系开发团队。
