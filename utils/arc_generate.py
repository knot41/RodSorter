import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # 导入3D绘图模块

def generate_arc_path(start, end, num_points, arc_height, avoid_radius):
    path = []
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    norm = np.hypot(dx, dy)
    line_dist = 0.0
    if norm > 1e-6:
        line_dist = abs(dx * start[1] - dy * start[0]) / norm

    for i in range(num_points + 1):
        t = i / num_points
        mid = np.zeros(6)
        # 线性插值
        mid[0] = start[0] + (end[0] - start[0]) * t
        mid[1] = start[1] + (end[1] - start[1]) * t
        # 抛物线插值z
        mid[2] = start[2] + (end[2] - start[2]) * t + arc_height * np.sin(np.pi * t)
        # 姿态
        mid[3] = 180
        mid[4] = 0
        mid[5] = start[5] + (end[5] - start[5]) * t

        # 外歪逻辑
        if avoid_radius > 0 and line_dist < avoid_radius and norm > 1e-6:
            nx = -dy / norm
            ny = dx / norm
            mid_x = start[0] + dx * t
            mid_y = start[1] + dy * t
            dot = mid_x * nx + mid_y * ny
            if dot < 0:
                nx = -nx
                ny = -ny
            offset = (avoid_radius - line_dist) * 0.5 * np.sin(np.pi * t)
            mid[0] += nx * offset
            mid[1] += ny * offset

        path.append(mid.copy())
    return np.array(path)

# 示例参数
end = [50, 200, 100, 180, 0, 0]
start = [100, -200, 100, 180, 0, 0]
num_points = 100
arc_height = 50
avoid_radius = 150

path = generate_arc_path(start, end, num_points, arc_height, avoid_radius)

# 绘图
fig = plt.figure(figsize=(9, 7)) # 创建一个图像
ax = fig.add_subplot(111, projection='3d') # 添加一个3D子图

# 绘制轨迹点
ax.plot(path[:, 0], path[:, 1], path[:, 2], 'o-', label='轨迹点')

# 绘制基坐标原点
ax.scatter([0], [0], [0], color='red', s=50, label='基坐标原点') # s是点的大小

# 绘制回避区（在XY平面上的圆柱底面示意）
theta = np.linspace(0, 2 * np.pi, 100)
x_circle = avoid_radius * np.cos(theta)
y_circle = avoid_radius * np.sin(theta)
# 你可以根据需要调整回避圆柱的高度范围，这里仅在z=0平面绘制
ax.plot(x_circle, y_circle, 0, color='orange', linestyle='--', label='回避区 (XY平面)')


# 设置坐标轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 设置图例和标题
ax.legend()
ax.set_title('三维样条轨迹点与回避区示意图')

# 可以尝试调整视角
# ax.view_init(elev=20., azim=-35)

plt.show()