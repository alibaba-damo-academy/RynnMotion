"""Spline interpolation test script."""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
x = np.array([0, 1, 2, 3, 4, 5])
y = np.array([0, 0.5, 2, 1.5, 3, 2.5])
t = np.linspace(0, 5, num=100)

# 创建原始样条
cs_original = CubicSpline(x, y, bc_type='clamped')
y_spline_original = cs_original(t)

# 定义起始位置和接入时间
start_position = np.array([0, 3])
connect_time_index = 30  # 假设在时间索引30处接入

# 选择用于过渡插值的几个时间点
transition_t_use = t[[0, connect_time_index // 2, connect_time_index]]

# 找到对应的 y 轴值，起点使用 start_position
mid_y_value = (start_position[1] + y_spline_original[connect_time_index]) / 2
transition_y_values = np.array([start_position[1], mid_y_value, y_spline_original[connect_time_index]])


# 设置过渡段的边界条件
start_velocity = 0
connect_velocity = cs_original(t[connect_time_index], 1)

# 创建过渡样条
cs_transition = CubicSpline(
    transition_t_use,
    transition_y_values,
    bc_type=((1, start_velocity), (1, connect_velocity))
)

# 计算过渡段的值
transition_y = cs_transition(t[:connect_time_index + 1])

# 合并过渡样条和原始样条
y_adjusted = np.concatenate((transition_y, y_spline_original[connect_time_index + 1:]))

# 绘制原始样条和调整后的样条
plt.figure(figsize=(12, 6))
plt.plot(t, y_spline_original, label='Original Cubic Spline')
plt.plot(t, y_adjusted, label='Adjusted Spline', linestyle='--')
plt.scatter(x, y, color='red', label='Data points')
plt.scatter([0], [3], color='green', label='Start Position')  # 起始位置标记
plt.title('Smooth Transition into Cubic Spline with Velocity Constraints')
plt.xlabel('Time')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()