"""学习制导拦截节点 —— uav_vision_png 的可替换实现

状态机/话题/QoS/CSV 统计格式与 vision_png_control.cpp 完全一致，
仅把 INTERCEPT 阶段的 PNG 计算替换为策略推理（policy_runtime）：

  ┌──────────────────────────────────────────────────────────────┐
  │  导引（纯视觉，严禁使用目标 GPS）                              │
  │    /camera_detect_result → 特征(15维) → GRU策略 → 速度指令     │
  │    watchdog 异常 → 回退内置 PNG（png_teacher 同一实现）        │
  ├──────────────────────────────────────────────────────────────┤
  │  统计记录（仅用于数据分析，不参与导引计算）                     │
  │    订阅 px4_2 GPS → 计算真实距离 → CSV（与 vpng 同列格式）     │
  └──────────────────────────────────────────────────────────────┘

定时器：50Hz 主控制循环（心跳+状态机，Offboard 流要求 >2Hz，50Hz 充裕），
        20Hz 策略推理（与训练 dt=0.05 一致）。
"""
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleStatus, VehicleOdometry, VehicleLocalPosition,
    SensorGps, HoverThrustEstimate,
)
from uav_common_msg.msg import RectMsg, Data as DataMsg

from .policy_runtime import PolicyRuntime
from guidance_rl.png_teacher import PNGTeacher
from guidance_rl.geometry import quat_to_euler

# WGS84 近似换算（统计用，精度对 <1km 场景足够；
# C++ 版用 GeographicLib，Python 侧避免引第三方依赖）
_M_PER_DEG_LAT = 111320.0


class State(Enum):
    TAKE_OFF = 0
    SEARCHING = 1
    INTERCEPT = 2
    TRACK_LOST = 3
    DONE = 4


class RLGuidanceNode(Node):
    def __init__(self):
        super().__init__("uav_rl_guidance")

        # ---- QoS（与 PX4 sensor_data 一致）----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # ---- 参数 ----
        self.declare_parameter("model_path",
                               "/home/verser/ros2_ws/src/uav_rl_guidance/models/policy.pt")
        self.declare_parameter("fallback_png", False)
        self.declare_parameter("focal_length", 1397.2)
        self.declare_parameter("image_width", 1920)
        self.declare_parameter("image_height", 1080)
        self.declare_parameter("kv", 4.0)
        self.declare_parameter("kz", 4.0)
        self.declare_parameter("speed_cmd", 5.0)
        self.declare_parameter("d_gain", 1.0)
        self.declare_parameter("k1_yaw", 0.0005)
        self.declare_parameter("k2_yaw", 0.0002)
        self.declare_parameter("k_ey", 0.012)
        self.declare_parameter("standby_altitude", -6.0)
        self.declare_parameter("coast_thresh", 30)
        self.declare_parameter("lost_thresh", 90)
        self.declare_parameter("hit_radius", 0.8)
        self.declare_parameter("csv_path", "/home/verser/ros2_ws/rl_intercept_stats.csv")
        # 台架调试：true 时跳过起飞直接进入 SEARCHING（无 PX4 验证检测→指令链路）
        self.declare_parameter("bench_test", False)

        g = lambda n: self.get_parameter(n).value
        self.fallback_png = bool(g("fallback_png"))
        self.standby_altitude = float(g("standby_altitude"))
        self.coast_thresh = int(g("coast_thresh"))
        self.lost_thresh = int(g("lost_thresh"))
        self.hit_radius = float(g("hit_radius"))
        self.csv_path = str(g("csv_path"))
        self.speed_cmd = float(g("speed_cmd"))

        # ---- PNG 回退控制器（参数与 uav_vision_png 一致）----
        # 节点按 60Hz 检测帧计数丢失（与 C++ 口径一致），
        # PNGTeacher 内部 coast/lost 由节点状态机接管 → 给一个大值禁用其内部分级
        png = PNGTeacher(
            focal=float(g("focal_length")),
            image_width=int(g("image_width")), image_height=int(g("image_height")),
            kv=float(g("kv")), kz=float(g("kz")),
            speed_cmd=self.speed_cmd, d_gain=float(g("d_gain")),
            k1_yaw=float(g("k1_yaw")), k2_yaw=float(g("k2_yaw")),
            k_ey=float(g("k_ey")),
            coast_steps=10**9, lost_steps=10**9,
        )

        # ---- 策略运行时 ----
        self.runtime = None
        if not self.fallback_png:
            model_path = str(g("model_path"))
            self.runtime = PolicyRuntime(model_path, png, logger=self.get_logger())
            self.get_logger().info(f"策略已加载: {model_path}")
        else:
            self.png = png
            self.get_logger().warn("fallback_png=true，全程使用内置 PNG（基线模式）")
        if self.runtime is None:
            self.png = png

        # ---- 发布者 ----
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/px4_1/fmu/in/offboard_control_mode", 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, "/px4_1/fmu/in/trajectory_setpoint", 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/px4_1/fmu/in/vehicle_command", 10)
        self.data_pub = self.create_publisher(DataMsg, "/vpng_data", 10)

        # ---- 订阅者：导引用 ----
        self.create_subscription(RectMsg, "/camera_detect_result",
                                 self.detect_cb, 10)
        self.create_subscription(VehicleOdometry,
                                 "/px4_1/fmu/out/vehicle_odometry",
                                 self.odom_cb, qos)
        self.create_subscription(VehicleLocalPosition,
                                 "/px4_1/fmu/out/vehicle_local_position",
                                 self.local_pos_cb, qos)
        self.create_subscription(VehicleStatus,
                                 "/px4_1/fmu/out/vehicle_status",
                                 self.status_cb, qos)
        self.create_subscription(HoverThrustEstimate,
                                 "/px4_1/fmu/out/hover_thrust_estimate",
                                 self.hover_cb, qos)

        # ---- 订阅者：纯统计用（导引算法严禁使用）----
        self.create_subscription(SensorGps,
                                 "/px4_1/fmu/out/vehicle_gps_position",
                                 self.self_gps_cb, qos)
        self.create_subscription(SensorGps,
                                 "/px4_2/fmu/out/vehicle_gps_position",
                                 self.target_gps_cb, qos)

        # ---- 状态变量 ----
        self.state = State.SEARCHING if bool(g("bench_test")) else State.TAKE_OFF
        if self.state == State.SEARCHING:
            self.get_logger().warn("bench_test=true：跳过起飞，直接 SEARCHING（仅台架调试）")
        self.detect = None            # (x, y, w, h) 或 None
        self.detect_seq = 0           # 检测帧序号（去重）
        self.consumed_seq = 0
        self.lost_frames = 0
        self.roll = self.pitch = self.yaw = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.local_z = 0.0
        self.hover_thrust_ok = False
        self.offboard_active = False
        self.takeoff_counter = 0
        self.takeoff_frames = 125     # 50Hz × 2.5s（同 C++ 200Hz×500）
        self.coast_cmd = (0.0, 0.0, 0.0, 0.0)
        self.last_cmd_source = "png" if self.fallback_png else "policy"
        self.intercept_entered = False

        # ---- 统计（不参与导引）----
        self.gps_origin = None        # (lat, lon, alt)
        self.self_pos = None          # NED（统计坐标系）
        self.target_pos = None
        self.min_distance = 1e9
        self.hit_recorded = False
        self.total_detect_frames = 0
        self.lost_frames_total = 0
        self.csv_file = None

        # ---- 定时器 ----
        self.timer_control = self.create_timer(0.02, self.control_loop)   # 50 Hz
        self.timer_policy = self.create_timer(0.05, self.policy_tick)     # 20 Hz

        self.get_logger().info(
            f"RLGuidance 节点已启动 | fallback_png={self.fallback_png} "
            f"standby={self.standby_altitude} hit_r={self.hit_radius}")

    # ==================================================================
    #  回调
    # ==================================================================
    def detect_cb(self, msg: RectMsg):
        if msg.width == -1:
            self.detect = None
            self.lost_frames += 1
            self.lost_frames_total += 1
        else:
            self.detect = (msg.x, msg.y, msg.width, msg.height)
            self.lost_frames = 0
            if self.state == State.SEARCHING:
                self.get_logger().info("→ 视觉检测到目标，进入 INTERCEPT")
                self._enter_intercept()
            elif self.state == State.TRACK_LOST:
                self.get_logger().info("→ 目标重新出现，恢复 INTERCEPT")
                self._enter_intercept()
        self.detect_seq += 1
        self.total_detect_frames += 1

    def odom_cb(self, msg: VehicleOdometry):
        q = msg.q
        self.roll, self.pitch, self.yaw = quat_to_euler(q[0], q[1], q[2], q[3])

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.vx, self.vy, self.vz = msg.vx, msg.vy, msg.vz
        self.local_z = msg.z

    def status_cb(self, msg: VehicleStatus):
        self.offboard_active = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def hover_cb(self, msg: HoverThrustEstimate):
        if msg.hover_thrust_var < 0.0025:
            self.hover_thrust_ok = True

    def self_gps_cb(self, msg: SensorGps):
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m)
            self.get_logger().info(
                f"[统计] 坐标系原点初始化: lat={msg.latitude_deg:.7f} "
                f"lon={msg.longitude_deg:.7f}")
        self.self_pos = self._gps_to_ned(msg)

    def target_gps_cb(self, msg: SensorGps):
        if self.gps_origin is None:
            return
        self.target_pos = self._gps_to_ned(msg)

    def _gps_to_ned(self, msg):
        lat0, lon0, alt0 = self.gps_origin
        n = (msg.latitude_deg - lat0) * _M_PER_DEG_LAT
        e = (msg.longitude_deg - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0))
        d = -(msg.altitude_msl_m - alt0)
        return (n, e, d)

    # ==================================================================
    #  50 Hz 主控制循环（心跳 + 状态机）
    # ==================================================================
    def control_loop(self):
        if self.state == State.TAKE_OFF:
            self.handle_takeoff()
        elif self.state == State.SEARCHING:
            self.handle_searching()
        elif self.state == State.INTERCEPT:
            self.handle_intercept_heartbeat()
        elif self.state == State.TRACK_LOST:
            self.handle_track_lost()
        elif self.state == State.DONE:
            self.publish_offboard_velocity_mode()
            self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)

    def handle_takeoff(self):
        self.arm()
        self.set_offboard_mode()
        self.publish_offboard_position_mode()
        self.publish_position_setpoint(0.0, 0.0, self.standby_altitude)

        if self.takeoff_counter < self.takeoff_frames:
            self.takeoff_counter += 1
            return
        alt_err = abs(self.local_z - self.standby_altitude)
        if self.hover_thrust_ok and alt_err < 0.5:
            self.get_logger().info(
                f"→ 起飞完成 z={self.local_z:.2f}，进入 SEARCHING")
            self.state = State.SEARCHING

    def handle_searching(self):
        self.publish_offboard_velocity_mode()
        self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
        # 检测回调切换 INTERCEPT

    def handle_intercept_heartbeat(self):
        """50Hz：心跳 + 丢失分级；速度指令由 20Hz policy_tick 更新缓存"""
        self.arm()
        self.set_offboard_mode()

        if self.lost_frames >= self.lost_thresh:
            self.get_logger().warn(
                f"[INTERCEPT] 目标长时间丢失 lost={self.lost_frames}，切换 TRACK_LOST")
            self.state = State.TRACK_LOST
            return

        vx, vy, vz, yaw_rate = self.coast_cmd
        self.publish_offboard_velocity_mode()
        self.publish_velocity_setpoint(vx, vy, vz, yaw_rate)

    def handle_track_lost(self):
        """制动 + 慢旋搜索（vision_png handle_track_lost line 515-527）"""
        yaw_rate = self.coast_cmd[3]
        search_yaw = yaw_rate if abs(yaw_rate) > 0.01 else 0.2
        self.publish_offboard_velocity_mode()
        self.publish_velocity_setpoint(0.0, 0.0, 0.0, search_yaw)
        # 检测回调恢复 INTERCEPT

    # ==================================================================
    #  20 Hz 策略推理
    # ==================================================================
    def _enter_intercept(self):
        self.state = State.INTERCEPT
        if not self.intercept_entered:
            # 首次进入：重置策略隐藏状态与 PNG 状态
            if self.runtime is not None:
                self.runtime.reset()
            else:
                self.png.reset()
            self.intercept_entered = True

    def policy_tick(self):
        if self.state != State.INTERCEPT:
            return

        # 惯性续飞：短暂丢失（< coast_thresh 检测帧）保持缓存指令，
        # 超过则把 None 喂给策略（特征 valid=0/age 递增，策略学过 coast 行为）
        det = self.detect
        if det is None and self.lost_frames < self.coast_thresh:
            return  # 保持 coast_cmd 不变，由 50Hz 心跳继续发布

        if self.runtime is not None:
            vx, vy, vz, yaw_rate, source = self.runtime.step(
                det, self.roll, self.pitch, self.yaw,
                self.vx, self.vy, self.vz, self.local_z, 0.05)
            self.last_cmd_source = source
            fb = self.runtime.fb
        else:
            cmd = self.png.step(det, self.roll, self.pitch, self.yaw,
                                self.vx, self.vy, self.vz)
            vx, vy, vz, yaw_rate = cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate
            fb = None

        self.coast_cmd = (vx, vy, vz, yaw_rate)
        self.publish_debug_data(fb)
        self.save_stats_to_csv()

    # ==================================================================
    #  调试话题 /vpng_data（与 vpng 绘图脚本兼容）
    # ==================================================================
    def publish_debug_data(self, fb):
        msg = DataMsg()
        if fb is not None:
            msg.los_angle_v = float(fb.los_v)
            msg.los_angle_z = float(fb.los_z)
            msg.ex = float(fb.ex)
            msg.ey = float(fb.ey)
        msg.yaw = float(self.yaw)
        msg.pitch = float(self.pitch)
        msg.roll = float(self.roll)
        msg.vel_total = float(math.sqrt(
            self.vx ** 2 + self.vy ** 2 + self.vz ** 2))
        self.data_pub.publish(msg)

    # ==================================================================
    #  统计 CSV（列格式与 vpng_intercept_stats.csv 完全一致）
    # ==================================================================
    def save_stats_to_csv(self):
        if self.csv_file is None:
            try:
                self.csv_file = open(self.csv_path, "w")
            except OSError as e:
                self.get_logger().error(f"[统计] 无法打开 CSV: {e}")
                return
            self.csv_file.write(
                "time_s,self_x,self_y,self_z,target_x,target_y,target_z,"
                "dist_m,los_angle_v,los_angle_z,d_v_angle_v,d_v_angle_z,"
                "ex,ey,vx,vy,vz,detect_w,state\n")
            self.get_logger().info(f"[统计] CSV 已创建: {self.csv_path}")

        t = self.get_clock().now().nanoseconds * 1e-9
        dist = -1.0
        just_hit = False
        if self.self_pos is not None and self.target_pos is not None:
            dist = math.dist(self.self_pos, self.target_pos)
            self.min_distance = min(self.min_distance, dist)
            if dist < self.hit_radius and not self.hit_recorded:
                self.hit_recorded = True
                just_hit = True
                self.get_logger().warn(
                    f"★ [统计] 检测到命中！真实距离={dist:.3f} m (统计专用)")
                self.state = State.DONE

        # 节流：每 2 次写一行（与 C++ 一致）
        self._csv_counter = getattr(self, "_csv_counter", 0) + 1
        if self._csv_counter % 2 != 0 and not just_hit:
            return

        sp = self.self_pos or (0.0, 0.0, 0.0)
        tp = self.target_pos or (0.0, 0.0, 0.0)
        fb = self.runtime.fb if self.runtime is not None else None
        los_v = fb.los_v if fb else self.png.los_v
        los_z = fb.los_z if fb else self.png.los_z
        ex = fb.ex if fb else self.png.last_ex
        ey = fb.ey if fb else 0.0
        det_w = self.detect[2] if self.detect else -1
        self.csv_file.write(
            f"{t:.4f},{sp[0]:.4f},{sp[1]:.4f},{-sp[2]:.4f},"
            f"{tp[0]:.4f},{tp[1]:.4f},{-tp[2]:.4f},{dist:.4f},"
            f"{los_v:.4f},{los_z:.4f},0,0,"
            f"{ex:.4f},{ey:.4f},{self.vx:.4f},{self.vy:.4f},{self.vz:.4f},"
            f"{det_w},{self.state.value}\n")
        self.csv_file.flush()

    def close_csv(self):
        if self.csv_file is not None:
            self.csv_file.write("\n# ========== 统计摘要 ==========\n")
            self.csv_file.write(f"# 总检测帧数,{self.total_detect_frames}\n")
            self.csv_file.write(f"# 累计丢失帧数,{self.lost_frames_total}\n")
            self.csv_file.write(f"# 最近接距离(m),{self.min_distance}\n")
            if self.runtime is not None:
                self.csv_file.write(
                    f"# watchdog触发次数,{self.runtime.watchdog_total}\n")
            self.csv_file.close()
            self.csv_file = None
            self.get_logger().info(f"[统计] CSV 已保存至: {self.csv_path}")

    # ==================================================================
    #  PX4 发布辅助（与 vision_png_control.cpp line 613-674 一致）
    # ==================================================================
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def publish_offboard_position_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def publish_offboard_velocity_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(self.yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.traj_pub.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, vz, yaw_rate):
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = math.nan
        msg.yawspeed = float(yaw_rate)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.command = command
        msg.target_system = 2     # px4_1 (Instance 1 → DDS key 2)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RLGuidanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close_csv()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
