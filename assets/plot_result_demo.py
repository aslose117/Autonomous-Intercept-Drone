import numpy as np
import matplotlib
from matplotlib import rcParams
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ==========================================
# 1. 全局绘图风格设置 (论文发表级配置)
# ==========================================
# config = {
#     "font.family": 'serif', # 使用衬线体
#     "font.serif": ['Times New Roman'], # 指定为 Times New Roman
#     "font.size": 14,        # 全局字号
#     "axes.labelsize": 16,   # 坐标轴标签字号
#     "axes.titlesize": 16,   # 标题字号
#     "xtick.labelsize": 14,  # x轴刻度字号
#     "ytick.labelsize": 14,  # y轴刻度字号
#     "axes.linewidth": 1.5,  # 坐标轴边框粗细
#     "grid.linewidth": 1.0,  # 网格线粗细
#     "lines.linewidth": 2.5, # 线条粗细 (关键：加粗线条)
#     "legend.fontsize": 14,  # 图例字号
#     "figure.figsize": (10, 14), # 图片比例 (竖向长图)
#     "toolbar": "none"           #禁用工具栏
# }

config = {
    "font.family": 'serif', 
    # 关键修改：添加备选字体。如果找不到 Times New Roman，自动使用 DejaVu Serif 或 Liberation Serif
    "font.serif": ['Times New Roman', 'DejaVu Serif', 'Liberation Serif', 'serif'],
    "mathtext.fontset": "stix", # 关键修改：让数学公式(如 a_n) 使用内置的类 Times 字体，无需安装
    "font.size": 14,        
    "axes.labelsize": 16,   
    "axes.titlesize": 16,   
    "xtick.labelsize": 14,  
    "ytick.labelsize": 14,  
    "axes.linewidth": 1.5,  
    "grid.linewidth": 1.0,  
    "lines.linewidth": 2.5, 
    "legend.fontsize": 14,  
    "figure.figsize": (10, 14) 
}
rcParams.update(config)

# ==========================================
# 2. 生成模拟数据 (与之前逻辑一致，增加清晰度)
# ==========================================
# 时间轴
time = np.linspace(0, 6.2, 620)

# --- 数据生成 ---
# (c) 像素轨迹数据：模拟目标在画面中心(320,240)附近的晃动
np.random.seed(42) # 固定随机种子，保证每次画出来都一样
u_noise = np.cumsum(np.random.normal(0, 1.5, len(time)))
v_noise = np.cumsum(np.random.normal(0, 1.5, len(time)))
u_pixel = 380 + u_noise  # 稍微偏离中心一点，让图更好看
v_pixel = 260 + v_noise

# (d) 法向加速度数据：模拟震动
acc_base = 2.0 + 0.5 * np.sin(time * 3)
acc_noise = np.random.normal(0, 0.8, len(time)) # 稍微减小噪音幅度，让图看起来更整洁但仍有真实感
acc_normal = np.abs(acc_base + acc_noise)

# (e) 速度数据
vel_total = 6.0 * (1 - np.exp(-0.4 * time)) + 0.2 * time  # 黄线
vel_normal = 1.5 * np.sin(time/2) * np.exp(-0.3*time)     # 绿线 (法向速度)
vel_normal = np.abs(vel_normal)

# ==========================================
# 3. 开始绘图
# ==========================================
fig = plt.figure()

# --- 子图 1: (c) 像素轨迹 ---
ax1 = fig.add_subplot(3, 1, 1)
# 画红色粗线
ax1.plot(u_pixel, v_pixel, color='#D95319', label='Trajectory') 
# 绘制类似示波器的网格
ax1.grid(True, which='major', linestyle='--', alpha=0.6, color='gray')
# 设置坐标范围 (模拟相机分辨率)
ax1.set_xlim(350, 420) 
ax1.set_ylim(240, 320)
ax1.invert_yaxis() # 像素坐标系Y轴向下
# 设置标签
ax1.set_xlabel('u (pixel)', fontweight='bold')
ax1.set_ylabel('v (pixel)', fontweight='bold')
# 添加左上角的标号 (c)
ax1.text(-0.1, 1.05, '(c)', transform=ax1.transAxes, size=20, weight='bold')

# --- 子图 2: (d) 法向加速度 ---
ax2 = fig.add_subplot(3, 1, 2)
# 画橙色线
ax2.plot(time, acc_normal, color='#D95319') 
ax2.grid(True, which='major', linestyle='--', alpha=0.6, color='gray')
ax2.set_xlim(0, 6.2)
ax2.set_ylim(0, 8)
ax2.set_ylabel('a$_n$ (m/s$^2$)', fontweight='bold') # 支持LaTeX格式下标
# 添加左上角的标号 (d)
ax2.text(-0.1, 1.05, '(d)', transform=ax2.transAxes, size=20, weight='bold')
# 添加内部图例框
ax2.legend(['a$_n$'], loc='upper left', frameon=True, edgecolor='black')

# --- 子图 3: (e) 速度曲线 ---
ax3 = fig.add_subplot(3, 1, 3)
# 画黄色线 (总速度)
ax3.plot(time, vel_total, color='#EDB120', label='||V||')
# 画绿色线 (法向速度)
ax3.plot(time, vel_normal, color='#77AC30', label='V$_n$')
ax3.grid(True, which='major', linestyle='--', alpha=0.6, color='gray')
ax3.set_xlim(0, 6.2)
ax3.set_ylim(0, 7)
ax3.set_xlabel('Time (s)', fontweight='bold')
ax3.set_ylabel('Velocity (m/s)', fontweight='bold')
# 添加左上角的标号 (e)
ax3.text(-0.1, 1.05, '(e)', transform=ax3.transAxes, size=20, weight='bold')
# 图例
ax3.legend(loc='upper left', frameon=True, edgecolor='black')

# ==========================================
# 4. 保存与显示
# ==========================================
plt.tight_layout() # 自动调整间距，防止重叠
plt.subplots_adjust(hspace=0.3) # 稍微增加子图之间的垂直距离

# 保存为高清图片 (300 DPI 是打印标准)
save_path = './flight_experiment_results.png'
plt.savefig(save_path, dpi=300, bbox_inches='tight')

print(f"图片已生成并保存为: {save_path}")
plt.show()