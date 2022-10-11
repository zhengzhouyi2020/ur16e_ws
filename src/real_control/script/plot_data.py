#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/7/4
# @Author : Zzy
import numpy as np
import rtde_receive
from matplotlib import pyplot as plt


def data_plot(ax, x, y, xlabel, ylabel, title="", color='r', is_grid=False):

    ax.plot(x, y, color=color, linestyle='-')
    ax.set_title(title)
    ax.spines['right'].set_visible(False)  # 设置右侧坐标轴不可见
    ax.spines['top'].set_visible(False)  # 设置上坐标轴不可见
    ax.spines['top'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.spines['right'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.set_xlabel(xlabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离
    ax.set_ylabel(ylabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离


file_name = "../data/202207182115_real_robot_m400_b5000_k8_r1600_f15.csv"
data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)

index = 14
l = np.array(data)
length = [i * 0.02 for i in range(len(data))]
fig = plt.figure()
# ax1 = fig.add_subplot(231)
# data_plot(ax1, x=length, y=l[:, 0 + index], xlabel="step", ylabel="force_x  N", is_grid=True)
#
# ax2 = fig.add_subplot(232)
# data_plot(ax2, length, l[:, 1 + index], "step", "force_y  N")
#
# ax3 = fig.add_subplot(233)
# data_plot(ax3, length, l[:, 2 + index], "step", "force_z  N")
#
# ax4 = fig.add_subplot(234)
# data_plot(ax4, length, l[:, 3 + index], "step", "force_x  N")
#
# ax5 = fig.add_subplot(235)
# data_plot(ax5, length, l[:, 4 + index], "step", "force_x  N")
#
# ax6 = fig.add_subplot(236)
# data_plot(ax6, length, l[:, 5 + index], "step", "force_x  N")


ax1 = fig.add_subplot(111)
data_plot(ax1, length, l[:, 2 + index],"step", "force_z  N", title= "m400_b5000_k8_r1600_f15")

plt.show()
