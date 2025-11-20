import numpy as np
from scipy.interpolate import BSpline, make_interp_spline
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt

# 机械臂的关键点（可根据实际情况修改）
waypoints = np.array([
    [0, 0],
    [1, 2],
    [3, 3],
    [4, 0],
    [5, 2]
])

# 提取x和y坐标
x = waypoints[:, 0]
y = waypoints[:, 1]

# 参数化
t = np.linspace(0, 1, len(waypoints))

# 生成B样条曲线（3阶）
spl_x = make_interp_spline(t, x, k=3)
spl_y = make_interp_spline(t, y, k=3)

# 生成插值点
t_new = np.linspace(0, 1, 200)
# 假设已知起点和终点，三维坐标
start = np.array([0, 0, 0])
end = np.array([5, 2, 0])

# 插值中间点数量
num_mid_points = 3

# 线性插值生成中间点
mid_points = [start + (end - start) * (i + 1) / (num_mid_points + 1) for i in range(num_mid_points)]
mid_points = np.array(mid_points)

# 给中间点的z坐标加一个向上弧度（如0.5, 0.8, 0.5）
arc_z = np.array([0.5, 0.8, 0.5])
mid_points[:, 2] += arc_z

# 合并所有点
waypoints = np.vstack([start, mid_points, end])

# 提取x, y, z
x = waypoints[:, 0]
y = waypoints[:, 1]
z = waypoints[:, 2]

# 参数化
t = np.linspace(0, 1, len(waypoints))

# 生成B样条曲线（3阶）
spl_x = make_interp_spline(t, x, k=3)
spl_y = make_interp_spline(t, y, k=3)
spl_z = make_interp_spline(t, z, k=3)

# 生成插值点
t_new = np.linspace(0, 1, 200)
x_new = spl_x(t_new)
y_new = spl_y(t_new)
z_new = spl_z(t_new)

# 绘制三维轨迹
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_new, y_new, z_new, label='B样条路径', color='blue')
ax.plot(x, y, z, 'ro--', label='关键点')
ax.set_title('机械臂三维空间B样条路径规划')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()
y_new = spl_y(t_new)

# 绘图
plt.figure(figsize=(8, 6))
plt.plot(x_new, y_new, label='B样条路径', color='blue')
plt.plot(x, y, 'ro--', label='关键点')
plt.title('机械臂笛卡尔空间B样条路径规划')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()