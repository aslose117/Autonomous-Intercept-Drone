import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader # 
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import numpy as np

# 假设你的 bag 路径
BAG_PATH = './rosbag2_2026_xx_xx' 
TOPIC_NAME = '/los_data'

# 存储数据
times = []
u_pixels = []
v_pixels = []
acc_normals = []
vel_totals = []
vel_normals = []

# 读取 Bag 文件
with Reader(BAG_PATH) as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == TOPIC_NAME:
            # 反序列化 (根据你的 msg 定义，这里需要你有编译好的 python msg 库或者手动解析)
            # 这里假设你已经生成了 Python 消息类，或者手动结构解析
            # 为简化，这里用伪代码表示 msg 对象获取
            msg = deserialize_cdr(rawdata, connection.msgtype)
            
            times.append(timestamp / 1e9) # 转换为秒
            u_pixels.append(msg.pixel_u)
            v_pixels.append(msg.pixel_v)
            acc_normals.append(msg.acc_normal)
            vel_totals.append(msg.vel_total)
            vel_normals.append(msg.vel_normal)

# 归一化时间 (从 0 开始)
times = np.array(times)
times = times - times[0]

# --- 绘图 (仿照论文风格) ---
fig = plt.figure(figsize=(12, 10))

# 图 (c): 像素轨迹 (Pixel Trajectory)
ax_c = fig.add_subplot(3, 1, 1)
ax_c.plot(u_pixels, v_pixels, 'r-', linewidth=2)
ax_c.set_title('(c) Target in FOV')
ax_c.set_xlabel('u (pixel)')
ax_c.set_ylabel('v (pixel)')
ax_c.set_xlim([0, 640]) # 假设图像宽 640
ax_c.set_ylim([0, 480]) # 假设图像高 480
ax_c.invert_yaxis() # 图像坐标系通常 y 向下
ax_c.grid(True)

# 图 (d): 法向加速度 (Normal Acceleration)
ax_d = fig.add_subplot(3, 1, 2)
ax_d.plot(times, acc_normals, 'tab:orange', label='a_n')
ax_d.set_title('(d) Normal Acceleration')
ax_d.set_ylabel('a_n (m/s^2)')
ax_d.set_xlabel('Time (s)')
ax_d.legend()
ax_d.grid(True)

# 图 (e): 速度曲线 (Velocity)
ax_e = fig.add_subplot(3, 1, 3)
ax_e.plot(times, vel_totals, 'y-', linewidth=2, label='||V||')
ax_e.plot(times, vel_normals, 'g-', linewidth=2, label='V_n')
ax_e.set_title('(e) Velocity')
ax_e.set_ylabel('V (m/s)')
ax_e.set_xlabel('Time (s)')
ax_e.legend()
ax_e.grid(True)

plt.tight_layout()
plt.show()