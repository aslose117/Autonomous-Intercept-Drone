//#include <iostream>
#include <rclcpp/rclcpp.hpp>
//  发送控制信号
#include <px4_msgs/msg/offboard_control_mode.hpp>
//  轨迹点话题
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/input_rc.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include "uav_common_msg/msg/rect_msg.hpp"

#include "los_control_data/msg/data.hpp"
//	姿态数据信息
#include <px4_msgs/msg/vehicle_odometry.hpp>
//	速度位置信息
#include "px4_msgs/msg/vehicle_local_position.hpp"
//	姿态及升力数据设定信息
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/hover_thrust_estimate.hpp"
#include "ctime"
#include <chrono>
#include <iostream>
#include <cmath>
#include <algorithm>
//	互斥锁
#include "mutex"
#include <Eigen/Dense>
using namespace Eigen;
using namespace px4_msgs::msg;
using namespace std::chrono;


std::string msg_name="/px4_2/fmu";

enum class State { take_off,image_detect,tracking,detcte_loss};

class uav_chase : public rclcpp::Node
{
public:
	~uav_chase()
	{
		std::cout<<"Mission End"<<std::endl;
		printf("control_time: %f\n",control_time);
		printf("control_lose_time: %f\n",control_lose_time);
		printf("proportion_time: %f\n",control_lose_time/control_time);
	}
    //  构造函数，当使用类初始化对象时自动执行
    uav_chase():Node("uav_chase")
    {
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			//创建控制模式发布者
			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			//创建轨迹点模式发布者
			trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
			//控制命令发布者
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
			//创建轨迹及升力控制
			vehicle_rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);
			//创建绘图数据发布者
			data_plot_publisher_ = this->create_publisher<los_control_data::msg::Data>("/los_data", 10);

			//创建悬浮升力监听者
			hover_thrust_subscription_ = this->create_subscription<HoverThrustEstimate>("/fmu/out/hover_thrust_estimate", qos,
			[this](const HoverThrustEstimate::SharedPtr msg)
			{
				if(msg->hover_thrust_var<0.0025)
				{this->hover_thrust = msg->hover_thrust;
				hover_flag = true;}
			});
			//	创建 识别信息话题接收者
			track_result_subscription_ = this->create_subscription<uav_common_msg::msg::RectMsg>("/uav_detect_result", 10,
			[this](const uav_common_msg::msg::RectMsg::UniquePtr msg)
			{
				this->x = msg->x;
				this->y = msg->y;
				this->uav_width = msg->width;
				this->uav_height = msg->height;
				this->tracke_msg_flag = true;
				track_time +=1;
				if(msg->width == -1)
				{
					lost_tarck +=1;
				}else
				{
					lost_tarck = 0;
					state = State::tracking;
				}

				auto now = std::chrono::system_clock::now();
				track_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
			});
			//	创建 姿态信息话题接收者
			vehicle_odometry_subscription_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
			[this](const VehicleOdometry::SharedPtr msg)
			{
				double q0, q1, q2, q3;
				q0 = msg->q[0];
				q1 = msg->q[1];
				q2 = msg->q[2];
				q3 = msg->q[3];
				this->roll  = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
				this->pitch = std::asin(2 * (q0 * q2 - q3 * q1));
				this->yaw = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
			});
			//	创建 速度信息话题接收者
			local_position_subscription_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
			[this](const VehicleLocalPosition::SharedPtr msg)
			{
				this->vx = msg->vx;
				this->vy = msg->vy;
				this->vz = msg->vz;
				local_position_z = msg->z;
			});
			//	创建 无人机状态话题接受者
			VehicleState_subscription_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos,
			[this](const VehicleStatus::SharedPtr msg)
			{
				if(msg->nav_state == msg->NAVIGATION_STATE_OFFBOARD)
				{
					this->offboard_state = true;
				}
			});
		}
			//	定时器回调函数
			auto timer_callback = [this]() -> void {
			missionfunction();
			};
			//	创建一个100Hz的定时器
			timer_ = this->create_wall_timer(5ms, timer_callback);

			auto LOS_timer_callback = [this]() -> void {
				if(lost_tarck<1)
				{
					LOS_calculate();
				}
			};

			timer_LOS = this->create_wall_timer(20ms, LOS_timer_callback);
	}

private:
	//	各类函数
	//  解锁无人机
	void arm();
	//	锁定无人机
	void disarm();
	//	发布控制模式
	void publish_offboard_control_mode();
	//  发布轨迹控制
	void publish_trajectory_setpoint(float x,float y,float z,float yaw);
	//  发布速度控制
	void publish_trajectory_velocity(float x,float y,float z,float yaw);
	//  发布控制命令
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	//  发布轨迹控制
	void take_off();
	//	
	void publish_rates_setpoint(double roll, double pitch, double yaw, double thrust);
	void publish_offboard_control_mode_att();
	//	任务函数
	void missionfunction();
	void publish_offboard_control_mode_vel();
	//	等待识别
	void waiting_for_detect();
	//	追踪主体
	void tracker();
	//  追踪方法
	void LOS_function();
	//	悬停方法
	void hover();

	void decomposeVector(const Eigen::Vector3d& vector2de, const Eigen::Vector3d& LOS,Eigen::Vector3d& vector_parallel,Eigen::Vector3d& vector_perpendicular);

	void LOS_calculate();

	Eigen::Matrix3d cross_product(const Eigen::Vector3d& V);

	Eigen::Vector3d vex(const Eigen::Matrix3d& M);
	//  定时器
    rclcpp::TimerBase::SharedPtr timer_;
	//  LOS定时器
    rclcpp::TimerBase::SharedPtr timer_LOS;

    //  发布方
        //  发布控制模式
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	    //  发布轨迹控制
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        //  发布
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		//	发布姿态以及升力数据
	rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;
		//  发布绘图数据
	rclcpp::Publisher<los_control_data::msg::Data>::SharedPtr data_plot_publisher_;
    
	//  订阅者
	//	无人机状态订阅者
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr VehicleState_subscription_;
	//	GPS订阅者
	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr GPS_subscription_;
	//	追踪数据订阅
	rclcpp::Subscription<uav_common_msg::msg::RectMsg>::SharedPtr track_result_subscription_;
	//	姿态数据订阅
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscription_;
	//	RC遥控器数据订阅
	//	rclcpp::Subscription<InputRc>::SharedPtr RC_data_subscription_;
	//	速度数据订阅
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
	//	升力订阅者
	rclcpp::Subscription<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_subscription_;
	//	悬浮升力订阅者
	rclcpp::Subscription<HoverThrustEstimate>::SharedPtr hover_thrust_subscription_;
	//	升力数据存储器
	std::vector<float> thrust_vector;

	//	高度测量
	float local_position_z = 0;
	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	std::atomic<uint64_t> old_track_timestamp_;
	std::atomic<uint64_t> new_track_timestamp_; 
	uint64_t offboard_setpoint_counter_ = 0;   //!< counter for the number of setpoints sent
	float standby_height = 1;
	double lost_tarck = 1;
	double a_max;
	//	计算过程中所需要使用到的各种信息
	//	焦距
	double foc = 554;
	
	//	接收的无人机数据
	double x = 0;
	double y = 0;
	double image_width = 1920;
	double image_height = 1080;
	double uav_width = 0;
	double uav_height = 0;
	double roll = 0;
	double pitch = 0;
	double yaw = 0;
	double vx = 0;
	double vy = 0;
	double vz = 0;

	//	LOS控制方法使用的变量
	//	FOV保持器
	double d_yaw = 0;
	double ex = 0;
	double ey = 0;
	double last_ex = 0;
	double k1 = 0.003;
	double k2 = 0.002;
	//	速度控制
	double d_v	= 1;
	double d_gain	= 1;
	double kv	= 5;
	double kz	= 5;

	double d_vx = 0;
	double d_vy = 0;
	double d_vz = 0;

	double v_angle_v = 0;
	double v_angle_z = 0;

	double LOS_angle_v = 0;
	double LOS_angle_z = 0;

	double d_v_angle_v = 0;
	double d_v_angle_z = 0;

	double last_v_angle_v = 0;
	double last_v_angle_z = 0;

	double last_LOS_angle_v = 0;
	double last_LOS_angle_z = 0;

	double LOS_counter = 0;

	// 质量大小
	double m = 0.945;

	double real_thrust = 0;

	double thrust_proportion = 0;

	double hover_thrust = 0;

	Eigen::Vector3d Nt;

	Eigen::Vector3d last_Nt;
	//	获取速度向量
	Eigen::Vector3d Vt;

	Eigen::Vector3d d_a;

	Eigen::Vector3d d_thrust;

	Eigen::Vector3d d_g;
	
	Eigen::Vector3d d_thrust_direction;

	Eigen::Vector3d last_d_thrust_direction;
	//	单位矩阵
	Eigen::Matrix3d I3=Eigen::Matrix3d::Identity();

	Eigen::Vector3d rotationAxis;

	Eigen::Matrix3d d_rotation_M;

	Eigen::Vector3d d_w;

	Eigen::Vector3d body_z_axis;

	double rotationAngle=0;

	Eigen::Vector3d d_Vt = Eigen::Vector3d::Zero();

	Eigen::Vector3d v_parallelpar;

	Eigen::Vector3d v_perpendicular;

	Eigen::Vector3d a_parallel;

	Eigen::Vector3d a_perpendicular;

	// 添加用于存储角度数据的容器
	std::vector<double> los_angle_v_data;
	std::vector<double> los_angle_z_data;
	std::vector<double> vt_angle_v_data;
	std::vector<double> vt_angle_z_data;
	std::vector<double> d_vt_angle_v_data;
	std::vector<double> d_vt_angle_z_data;

	//	标志位
	bool tracke_msg_flag = false;
	bool uav_msg_flag = false;
	bool start_flag = true;
	double track_time = 0;
	bool offboard_state = false;
	long long track_timestamp;
	long long control_timestamp;
	long long time_diff;
	//	LOS标志位 是否初次进入LOS控制
	bool LOS_flag = false;	

	bool hover_flag = false;

	double control_time = 0;

	double control_lose_time = 0;
	
	//	时间标志
	double test_time=0;

	//	任务状态机变量
	State state = State::take_off;
};
int main(int argc, char *argv[]){
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uav_chase>());
    rclcpp::shutdown();
    return 0;
}
void uav_chase::LOS_calculate()
{
	//	计算LOS向量
	ex =  x+uav_width/2 - image_width/2;
	ey =  y+uav_height/2 - image_height/2;
	Nt << ex,ey,foc;
	//	计算重力向量
	d_g << 0,0,9.81;
	//	获得升力朝向
	body_z_axis << 0,0,-1;

	// 获得从BCS旋转至ENU坐标系下的旋转矩阵
	// 绕Z轴旋转
	Eigen::AngleAxisd b2nrotationYaw(yaw, Eigen::Vector3d(0, 0, 1)); 
	// 绕Y轴旋转
	Eigen::AngleAxisd b2nrotationPitch(pitch, Eigen::Vector3d(0, 1, 0)); 
	// 绕X轴旋转
    Eigen::AngleAxisd b2nrotationRoll(roll, Eigen::Vector3d(1, 0, 0)); 
	// 合成旋转矩阵：先偏航（yaw），后俯仰（pitch），再滚转（roll）
    Eigen::Matrix3d b2nrotationMatrix = b2nrotationYaw.toRotationMatrix() 
								* b2nrotationPitch.toRotationMatrix() 
								* b2nrotationRoll.toRotationMatrix();
	
	// 获得从CCS旋转至BCS坐标系下的矩阵
	// 绕Z轴旋转
	Eigen::Matrix3d c2brotation;
	c2brotation << 	0, 0, 1,  // Cx -> By
                	1, 0, 0,  // Cy -> Bz
                 	0, 1, 0;  // Cz -> Bx

	//	计算LOS的v平面和z平面角度
	//	将LOS矢量从CCS旋转至BCS坐标系
	Nt = Nt;
	Nt = c2brotation * Nt;
	//	将LOS矢量从BCS旋转至NED坐标系
	Nt = b2nrotationMatrix * Nt;

	LOS_angle_v = atan2(Nt[2],sqrt(Nt[1]*Nt[1]+Nt[0]*Nt[0]));
	LOS_angle_z = atan2(Nt[0],Nt[1]);

	double diff_LOS_angle_v = LOS_angle_v-last_LOS_angle_v;
	double diff_LOS_angle_z = LOS_angle_z-last_LOS_angle_z;
	// printf("diff_LOS_angle_v: %f\n",LOS_angle_v-last_LOS_angle_v);
	// printf("diff_LOS_angle_z: %f\n",LOS_angle_z-last_LOS_angle_z);

	control_time += 1;
	if ((last_LOS_angle_v==0&&last_LOS_angle_z==0)||Vt.norm()<1)
	{
		d_v_angle_v = LOS_angle_v;
		d_v_angle_z = LOS_angle_z;

		last_LOS_angle_v = LOS_angle_v;
		last_LOS_angle_z = LOS_angle_z;

		last_v_angle_v = atan2(Vt[2],sqrt(Vt[1]*Vt[1]+Vt[0]*Vt[0]));
		last_v_angle_z = atan2(Vt[0],Vt[1]);
	
		last_Nt = Nt;
	}else
	{	
		// 检测是否与上一刻的相同
		if(Nt == last_Nt)
		{
			control_lose_time += 1;
		}
		last_Nt = Nt;
		if(abs(diff_LOS_angle_v)<0.02)
		{

		}
		else
		{
			last_v_angle_v = atan2(Vt[2],sqrt(Vt[1]*Vt[1]+Vt[0]*Vt[0]));
			last_v_angle_z = atan2(Vt[0],Vt[1]);

			// d_v_angle_v = kv*(LOS_angle_v - last_LOS_angle_v) + LOS_angle_v;
			// d_v_angle_z = kz*(LOS_angle_z - last_LOS_angle_z) + LOS_angle_z;

			d_v_angle_v = kv*(LOS_angle_v - last_LOS_angle_v) + last_v_angle_v;
			d_v_angle_z = kz*(LOS_angle_z - last_LOS_angle_z) + last_v_angle_z;
			
			// }
			// d_v_angle_v = kv*(LOS_angle_v - last_LOS_angle_v) + d_v_angle_v;
			// d_v_angle_z = kz*(LOS_angle_z - last_LOS_angle_z) + d_v_angle_z;
			// filter_diff_angle_v = filter_diff_angle_v+(1-filter_diff_v)*diff_LOS_angle_v;
			// filter_diff_angle_z = filter_diff_angle_z+(1-filter_diff_z)*diff_LOS_angle_z;

			// 计算当前速度角度
			printf("--------------------data-------------------\n");
			printf("last_v_angle_v: %f\n",last_v_angle_v);
			printf("d_v_angle_v: %f\n",d_v_angle_v);
			printf("LOS_angle_v: %f\n",LOS_angle_v);

			last_LOS_angle_v = LOS_angle_v;
			last_LOS_angle_z = LOS_angle_z;
			// last_v_angle_v = d_v_angle_v;
			// last_v_angle_z = d_v_angle_z;

			los_control_data::msg::Data data_msg{};
			data_msg.d_v_angle_v = d_v_angle_v;
			data_msg.d_v_angle_z = d_v_angle_z;
			data_msg.v_angle_v = atan2(Vt[2], sqrt(Vt[1]*Vt[1]+Vt[0]*Vt[0]));
			data_msg.v_angle_z = atan2(Vt[0], Vt[1]);
			data_msg.los_angle_v = LOS_angle_v;
			data_msg.los_angle_z = LOS_angle_z;
			data_msg.diff_los_angle_v = diff_LOS_angle_v;
			data_msg.diff_los_angle_z = diff_LOS_angle_z;
			data_msg.ex = ex;
			data_msg.ey = ey;
			data_plot_publisher_->publish(data_msg);
		}
	}

	// 使用LOS向量作为速度方向
	// d_Vt << cos(LOS_angle_v)*sin(LOS_angle_z), 
    //     cos(LOS_angle_v)*cos(LOS_angle_z), 
    //     sin(LOS_angle_v);

	
	d_Vt << cos(d_v_angle_v)*sin(d_v_angle_z), 
         cos(d_v_angle_v)*cos(d_v_angle_z), 
         sin(d_v_angle_v);
		 
	//	速度模长增益
	d_v = Vt.norm()+d_gain;

		//	计算FOV保持器
	if(last_ex == 0)
	{	d_yaw = 0;
		last_ex = ex;}
	else{d_yaw = k1*ex+k2*(ex-last_ex);
		last_ex = ex;}
	d_Vt = d_Vt*d_v;
}

void uav_chase::decomposeVector(const Eigen::Vector3d& vector2de, const Eigen::Vector3d& LOS,Eigen::Vector3d& vector_parallel,Eigen::Vector3d& vector_perpendicular)
{
	// 计算平行分量: proj_n(v) = (v·n / |n|²) * n
	double LOS_squared = LOS.squaredNorm(); // |n|²
	if (LOS_squared > 1e-10) { // 避免除零
		vector_parallel = (vector2de.dot(LOS) / LOS_squared) * LOS;
	} else {
		vector_parallel = Eigen::Vector3d::Zero();
	}
	
	// 计算垂直分量: v_perp = v - v_parallel
	vector_perpendicular = vector2de - vector_parallel;
}

/**
 * @brief cross product function
 */
Eigen::Matrix3d uav_chase::cross_product(const Eigen::Vector3d& V)
{
	Eigen::Matrix3d M;
	M << 0, -V(2), V(1),
		V(2), 0, -V(0),
		-V(1), V(0), 0;
	return M;
}
/**
 * @brief vex function
 */
Eigen::Vector3d uav_chase::vex(const Eigen::Matrix3d& M)
{
	Eigen::Vector3d V;
	V << M(2,1), M(0,2), M(1,0);
	return V;
}
/**
 * @brief take_off function
 */
void uav_chase::take_off()
{
	if (offboard_setpoint_counter_ == 500) {
	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	this->arm();}
	// SITL仿真
	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	this->arm();
	
	publish_offboard_control_mode();
	publish_trajectory_setpoint(0,0,-standby_height,-3.14);
	//publish_trajectory_setpoint(0,0,-standby_height,this->yaw);

	if (offboard_setpoint_counter_ < 501) {
		offboard_setpoint_counter_++;
	}
	// 切换状态
	else{ 
		if (hover_flag)
		{state = State::image_detect;}
		else
		{publish_offboard_control_mode();
		publish_trajectory_setpoint(0,0,-standby_height,-3.14);}
	}
}
/**
 * @brief waiting_for_detect
 */
void uav_chase::waiting_for_detect()
{ 
	// 如果丢失目标，进入到速度控制模式，保证不坠机 track_time要超过100的原因是识别部分前100帧不稳定
	if(lost_tarck > 60 || track_time <100){
		// publish_offboard_control_mode_vel();
		// publsh_trajectory_velocity(0,0,0,0);

		publish_offboard_control_mode();
		publish_trajectory_setpoint(0,0,-standby_height,-3.14);
	}
}
/**
 * @brief tracker
 *///	发布控制命令
void uav_chase::tracker()
{
	// 如果控制时间小于500ms则进入控制模式
	if(time_diff < 500)
	{
		//	如果没有丢失目标则进入控制模式
		if(lost_tarck<1)
		{this->LOS_function();}
		else if(lost_tarck<30)
		{
		//丢失目标小于30帧但是大于1帧则可能误识别，继续冲击
			if(Vt.norm()>1)
			{
				publish_offboard_control_mode_att();
				publish_rates_setpoint(d_w[0],d_w[1],d_w[2],-thrust_proportion);
			}
			else
			{
				publish_offboard_control_mode_vel();
				publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);
			}
		// publish_offboard_control_mode_vel();
		// publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);
		}
		else
		{std::cout<<"lost track!!!"<<std::endl;
		state = State::detcte_loss;
		}
	}
}
/**
 * @brief LOS_tracking
 */
void uav_chase::LOS_function()
{

	Vt << vx,vy,vz;
	// Attitude loop 姿态环控制
	// 获取期望加速度 Vt为NED坐标系 d_Vt为NED坐标系
	d_a =(d_Vt-Vt);
	d_g << 0,0,9.81;
	// 获取期望升力	 d_g为NED坐标系
	
	// 获得从BCS旋转至ENU坐标系下的旋转矩阵
	// 绕Z轴旋转
	Eigen::AngleAxisd b2nrotationYaw(yaw, Eigen::Vector3d(0, 0, 1)); 
	// 绕Y轴旋转
	Eigen::AngleAxisd b2nrotationPitch(pitch, Eigen::Vector3d(0, 1, 0)); 
	// 绕X轴旋转
    Eigen::AngleAxisd b2nrotationRoll(roll, Eigen::Vector3d(1, 0, 0)); 
	// 合成旋转矩阵：先偏航（yaw），后俯仰（pitch），再滚转（roll）
    Eigen::Matrix3d b2nrotationMatrix = b2nrotationYaw.toRotationMatrix() 
								* b2nrotationPitch.toRotationMatrix() 
								* b2nrotationRoll.toRotationMatrix();
	
	// 获得从CCS旋转至BCS坐标系下的矩阵
	// 绕Z轴旋转
	Eigen::Matrix3d c2brotation;
	c2brotation << 	0, 0, 1,  // Cx -> By
                	1, 0, 0,  // Cy -> Bz
                 	0, 1, 0;  // Cz -> Bx

	d_thrust = (d_a-d_g)*200;
	
	// 获取期望升力的方向向量 NED
	d_thrust_direction = d_thrust/d_thrust.norm();
	// 计算上次的升力方向 NED
	last_d_thrust_direction = b2nrotationMatrix*body_z_axis;
	//	获取旋转轴
	rotationAxis = last_d_thrust_direction.cross(d_thrust_direction);
	//	获取旋转轴的旋转角度
	rotationAngle = acos(last_d_thrust_direction.dot(d_thrust_direction));
	//	计算r的叉乘矩阵
	Eigen::Matrix3d r_cross_matrix = cross_product(rotationAxis);
	//	获取Rd
	d_rotation_M = (I3+r_cross_matrix*sin(rotationAngle)+r_cross_matrix*r_cross_matrix*(1-cos(rotationAngle)))*b2nrotationMatrix;
	//	计算角速度
	Eigen::Matrix3d temp = d_rotation_M.transpose()*b2nrotationMatrix-b2nrotationMatrix.transpose()*d_rotation_M;

	d_w = vex(temp);
	d_w = -1*d_w;
	d_w[2] = d_w[2]+d_yaw;
	// d_w[0] = 0;
	// if(d_w.norm()>8)
	// {d_w = d_w/d_w.norm()*8;}

	// real_thrust = std::min(std::max(last_d_thrust_direction.dot((d_a - m*d_g)),0.0),(m*9.81/hover_thrust));
	real_thrust = std::min(std::max(last_d_thrust_direction.dot((d_a - m*d_g)),0.0),(m*9.81/hover_thrust));

	thrust_proportion = real_thrust/(m*9.81/hover_thrust);
	
	if(Vt.norm()>1)
	{
		publish_offboard_control_mode_att();
		publish_rates_setpoint(d_w[0],d_w[1],d_w[2],-thrust_proportion);
	}
	else
	{
		publish_offboard_control_mode_vel();
		publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);
	}

	// publish_offboard_control_mode_att();
	// publish_rates_setpoint(d_w[0],d_w[1],d_w[2],-thrust_proportion);

	// publish_offboard_control_mode_vel();
	// publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);

	// publish_offboard_control_mode();
	// publish_trajectory_setpoint(0,0,-standby_height,-3.14);
}
/**
 * @brief uav hover functionpublish_offboard_control_mode_vel
 */
void uav_chase::hover()
{
	// 悬停
	publish_offboard_control_mode_vel();
	publish_trajectory_velocity(0,0,0,0);
}
/**
 * @brief missionfunction
 */
void uav_chase::missionfunction()
{
	// 获取当前时间点
	auto now = std::chrono::system_clock::now();
	control_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
	time_diff = control_timestamp - track_timestamp;
	
	switch (state)
	{
	case State::take_off:
		take_off();
		break;
	case State::image_detect: 
		waiting_for_detect();
		break;
	case State::tracking:
		tracker();
		break;
	case State::detcte_loss:
		hover();
		break;
	default:
		break;
	}

}
/**
 * @brief Send a command to control w and f
 */
 //发送命令控制无人机
void uav_chase::publish_rates_setpoint(double roll, double pitch, double yaw, double thrust)
{
    // 创建消息实例
	VehicleRatesSetpoint message{};
    message.roll = roll;
    message.pitch = pitch;
    message.yaw = yaw;
    message.thrust_body = {0,0,thrust};
    vehicle_rates_setpoint_publisher_->publish(message);
}
/**
 * @brief Send a command to Arm the vehicle
 */
 //发送命令对无人机进行解锁
void uav_chase::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
//发送命令使得无人机上锁
void uav_chase::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
//  发布控制模式，选择什么控制方式
void uav_chase::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	//	启用位置控制
	msg.position = true;
	//	禁用速度控制
	msg.velocity = false;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = false;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
//  发布控制模式，选择什么控制方式
void uav_chase::publish_offboard_control_mode_vel()
{
	OffboardControlMode msg{};
	//	启用位置控制
	msg.position = false;
	//	禁用速度控制
	msg.velocity = true;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = false;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
//  发布控制模式，选择什么控制方式
void uav_chase::publish_offboard_control_mode_att()
{
	OffboardControlMode msg{};
	//	启用速度控制
	msg.position = false;
	//	禁用速度控制
	msg.velocity = false;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = true;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
//  发布设点模式 控制无人机的点位
void uav_chase::publish_trajectory_setpoint(float x,float y,float z,float yaw)
{
	TrajectorySetpoint msg{};
	//	使用的是FRD坐标系
	msg.position = {x,y,z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
//  发布速度模式，控制无人机的速度
void uav_chase::publish_trajectory_velocity(float x,float y,float z,float yaw)
{
	TrajectorySetpoint msg{};
	//	使用的是FRD坐标系
	msg.position = {nan(""),nan(""),nan("")};
	msg.velocity = {x,y,z};
	msg.yaw = nan(""); // [-PI:PI]
	msg.yawspeed = yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
//  发布无人机命令
void uav_chase:: publish_vehicle_command(uint16_t command, float param1, float param2)
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
