# -*- coding: utf-8 -*-
# @Author  : zzy
# @FileName: splitData.py
# @Time    : 2021/10/15


import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D
import csv


#  等同于MATLAB的smooth函数, 但是平滑窗口必须为奇数
#  滑动平均,取滑动过程中的前后几个数作平均
def movingAverage(data, window_size):
    """
    :param data:  1-D数据
    :param window_size: 滑动平均的窗口,取奇数
    :return:
    """
    out0 = np.convolve(data, np.ones(window_size, dtype=int), 'valid') / window_size
    r = np.arange(1, window_size - 1, 2)
    start = np.cumsum(data[:window_size - 1])[::2] / r
    stop = (np.cumsum(data[:-window_size:-1])[::2] / r)[::-1]
    return np.concatenate((start, out0, stop))


# 要进行轨迹分割的文件
dataPath = r'20211215_robot_data_13_no_turn.csv'
with open(dataPath, encoding='utf-8') as f:
    trajectory = np.loadtxt(dataPath, delimiter=',', skiprows=1)
f.close()

# 每25取一个点,降低点的频率
trajectoryData = []
for i in range(len(trajectory)):
    if i % 1 == 0:
        trajectoryData.append(trajectory[i, :])

trajectoryData = np.array(trajectoryData)
timeStamp = trajectoryData[:, 0] - trajectoryData[0, 0]
Fx, Fy, Fz = trajectoryData[:, 14], trajectoryData[:, 15], trajectoryData[:, 16]
X, Y, Z = trajectoryData[:, 1], trajectoryData[:, 2], trajectoryData[:, 3]
movingAverage(X, 25)
movingAverage(Y, 25)
movingAverage(Z, 25)
Vx, Vy, Vz, V = [], [], [], []
ForceX, ForceY, ForceZ = [], [], []
time = []
XStart, YStart, ZStart = X[0], Y[0], Z[0]
timeStart = timeStamp[0]

for i in range(1, len(Fz)):
    Fz[i] = 0.5 * Fz[i] + 0.5 * Fz[i - 1]  # 一阶滤波

for i in range(1, len(timeStamp)):
    Vx.append((X[i] - XStart) / (timeStamp[i] - timeStart))
    Vy.append((Y[i] - YStart) / (timeStamp[i] - timeStart))
    Vz.append((Z[i] - ZStart) / (timeStamp[i] - timeStart))
    V.append(math.sqrt((Vx[i - 1] * Vx[i - 1] + Vy[i - 1] * Vy[i - 1] + Vz[i - 1] * Vz[i - 1])))
    ForceX.append((Fx[i] + Fx[i - 1]) / 2)
    ForceY.append((Fy[i] + Fy[i - 1]) / 2)
    ForceZ.append((Fz[i] + Fz[i - 1]) / 2)
    time.append((timeStamp[i] + timeStart) / 2)
    XStart, YStart, ZStart = X[i], Y[i], Z[i]
    timeStart = timeStamp[i]

firstForceZ, firstSpeed = ForceZ[0], V[0]
firstSplit, lastSplit = 0, len(ForceZ) - 1
forceZThreshold = np.mean(ForceZ)  # 设定力阈值
speedThreshold = np.mean(V) - 1 / 3 * np.std(V)  # 设定速度阈值

# 从前向后遍历
for i in range(0, len(ForceZ), 1):
    if ForceZ[i] < forceZThreshold and V[i] < speedThreshold:
        firstSplit = i
        break
# 从后向前遍历
for i in range(len(ForceZ) - 1, -1, -1):
    if (ForceZ[i] < forceZThreshold and V[i] < speedThreshold):
        lastSplit = i
        break

#  打印分割点
print("first split:{} last split:{},force threshold:{:.3f},speed threshold:{:.3f}".format(firstSplit, lastSplit,
                                                                                          forceZThreshold,
                                                                                          speedThreshold))

# 下面是图像显示
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(X[firstSplit:firstSplit + 1], Y[firstSplit:firstSplit + 1], Z[firstSplit:firstSplit + 1], color='r', marker='o',
        lineWidth=2)
ax.plot(X[lastSplit:lastSplit + 1], Y[lastSplit:lastSplit + 1], Z[lastSplit:lastSplit + 1], color='r', marker='o',
        lineWidth=2)
ax.plot(X, Y, Z, color='k', linewidth=2)
ax.set_title("Grinding Trajectory", fontSize=15)
ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')

fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
ax2.set_title("Grinding Speed", fontSize=15)
ax2.set_xlabel('t (s)')
ax2.set_ylabel('V (mm/s)')
ax2.plot(time, V, 'k')
ax2.plot(time[firstSplit:firstSplit + 1], V[firstSplit:firstSplit + 1], color='r', marker='o')
ax2.plot(time[lastSplit:lastSplit + 1], V[lastSplit:lastSplit + 1], color='r', marker='o')
plt.grid()

fig3 = plt.figure()
ax6 = fig3.add_subplot(111)
ax6.set_title("z-axis Contact Force", fontSize=15)
ax6.set_xlabel('t (s)')
ax6.set_ylabel('Fz (N)')
ax6.plot(time[firstSplit:firstSplit + 1], Fz[firstSplit:firstSplit + 1], color='r', marker='o')
ax6.plot(time[lastSplit:lastSplit + 1], Fz[lastSplit:lastSplit + 1], color='r', marker='o')
ax6.plot(time, ForceZ, 'k')
plt.grid()

plt.show()
