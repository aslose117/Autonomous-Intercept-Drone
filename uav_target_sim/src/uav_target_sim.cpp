#include "uav_target_sim.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <px4_msgs/msg/sensor_gps.hpp>

#include <fstream>    // std::ofstream
#include <iomanip>    // std::setprecision

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



UavTargetControl::UavTargetControl() : Node("uav_target_sim")
{
    // --- 发布者配置 ---
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_2/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_2/fmu/in/vehicle_command", 10);

    // --- 订阅者配置 (使用 Sensor Data QoS) ---
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    vehicle_rates_setpoint_subscription_ = this->create_subscription<VehicleRatesSetpoint>(
        "/px4_2/fmu/out/vehicle_rates_setpoint", qos,
        [this](const VehicleRatesSetpoint::SharedPtr msg) {
            // 频率较高，建议仅在调试时开启打印
            // std::cout << "Thrust: " << msg->thrust_body[2] << std::endl;
        });

    global_position_sub_ = this->create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        std::bind(&UavTargetControl::global_position_callback, this, std::placeholders::_1));

    // --- 状态变量初始化 ---
    offboard_setpoint_counter_ = 0;
    theta_ = 0.0;
    angular_speed_ = 0.05; // 弧度/周期 (10Hz下约 0.5 rad/s)

    // --- 定时器：核心逻辑控制 ---
    timer_ = this->create_wall_timer(100ms, [this]() {
        // 1. 必须始终发送 Offboard 模式心跳信号（否则 PX4 会退出该模式）
        publish_offboard_control_mode();

        if (offboard_setpoint_counter_ < 15) {
            // 阶段 A: 发送初始点，不切换模式 (PX4 要求在进入 Offboard 前必须有数据流)
            publish_trajectory_setpoint(5.0, 0.0, -5.0);
            offboard_setpoint_counter_++;
        } 
        else if (offboard_setpoint_counter_ == 15) {
            // 阶段 B: 只在第 15 次循环发送切换模式和解锁指令
            RCLCPP_INFO(this->get_logger(), "Switching to Offboard and Arming...");
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
            offboard_setpoint_counter_++;
        } 
        else {
            // 阶段 C: 成功进入后，执行圆周运动
            move_in_circle();
        }
    });
}

void UavTargetControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void UavTargetControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void UavTargetControl::publish_trajectory_setpoint(float x, float y, float z) {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = 0.0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void UavTargetControl::move_in_circle() {
    theta_ += angular_speed_;
    if (theta_ > 2 * M_PI) theta_ -= 2 * M_PI;

    float radius = 5.0;
    float new_x = radius * cos(theta_);
    float new_y = radius * sin(theta_);

    TrajectorySetpoint msg{};
    msg.position = {new_x, new_y, -5.0}; // NED 坐标系：-5 代表高度 5 米
    msg.yaw = theta_; // 让机头始终指向圆周切线或圆心方向（根据需求调整）
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void UavTargetControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 3;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void UavTargetControl::global_position_callback(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
    std::cout << "callback triggered!" << std::endl; 
    (void)msg; 
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null GPS message");
        return;
    }

    static bool initialized = false;
    static std::ofstream log_file;          // ← 只初始化一次

    // 打开文件（仅第一次执行时）
    if (!log_file.is_open()) {
        log_file.open("/home/verser/ros2_ws/src/uav_target_sim/xyz_log.txt", std::ios::out | std::ios::app);
        if (!log_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件！");
            return;
        }
        log_file << "timestamp, world_x, world_y, world_z\n";  // 写表头
    }

    px4_msgs::msg::SensorGps::SharedPtr target_sim_position = msg;
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

    RCLCPP_INFO(this->get_logger(), "gps_position:%f, %f, %f",
        target_sim_position->latitude_deg,
        target_sim_position->longitude_deg,
        target_sim_position->altitude_msl_m);

    if (!initialized) {
        earth.Forward(
            target_sim_position->latitude_deg,
            target_sim_position->longitude_deg,
            target_sim_position->altitude_msl_m,
            this->init_enu_xyz[0], this->init_enu_xyz[1], this->init_enu_xyz[2]);
        RCLCPP_INFO(this->get_logger(), "init_xyz:%f, %f, %f",
            this->init_enu_xyz[0], this->init_enu_xyz[1], this->init_enu_xyz[2]);
        initialized = true;
    }

    earth.Forward(
        target_sim_position->latitude_deg,
        target_sim_position->longitude_deg,
        target_sim_position->altitude_msl_m,
        this->enu_xyz[0], this->enu_xyz[1], this->enu_xyz[2]);

    double world_x = this->enu_xyz[0] - this->init_enu_xyz[0];
    double world_y = this->enu_xyz[1] - this->init_enu_xyz[1];
    double world_z = this->enu_xyz[2] - this->init_enu_xyz[2];

    RCLCPP_INFO(this->get_logger(), "world xyz: x=%f, y=%f, z=%f",
        world_x, world_y, world_z);

    // 获取当前时间戳（秒）并写入文件
    double timestamp = this->now().seconds();
    log_file << std::fixed << std::setprecision(6)
             << timestamp << ", "
             << world_x   << ", "
             << world_y   << ", "
             << world_z   << "\n";
    log_file.flush();   // ← 确保每次都实时写入磁盘
}