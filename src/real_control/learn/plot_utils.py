#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/11/2
# @Author : Zzy
import numpy as np
from matplotlib import pyplot as plt

plt.rcParams['font.family'] = 'Times New Roman'  # 全局字体样式
plt.rcParams['font.size'] = 15  # 全局字体大小
plt.rcParams['axes.linewidth'] = 1


# 在坐标轴中画单个表
def data_plot(ax, x, y, xlabel, ylabel, title="", color='r', is_grid=False):
    ax.plot(x, y, color=color, linestyle='-.', linewidth=0.8)
    ax.set_title(title, fontsize=9, )  # 设置标题
    ax.spines['right'].set_visible(False)  # 设置右侧坐标轴不可见
    ax.spines['top'].set_visible(False)  # 设置上坐标轴不可见
    ax.spines['top'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.spines['right'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.set_xlabel(xlabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离
    ax.set_ylabel(ylabel, fontsize=15, labelpad=5)  # 设置纵轴字体和距离坐标轴的距离
    # ax.set_ylim(0,10000)  #设置y轴范围
    # ax.set_xlim(0, 10000)  # 设置x轴范围
    # 添加网格
    if is_grid:
        ax.grid(which='major', ls='--', alpha=.8, lw=.5)  # 是否设置网格，设置网格宽度和形状
    # 设置刻度坐标的朝向
    # ax.tick_params(which='major', x=5, width=1.5, direction='in', top='on', right="on")
    # ax.tick_params(which='minor', x=3, width=1, direction='in', top='on', right="on")

force_list = []
reward_list = []
# 绘图,绘制接触力的图
l = np.array(force_list)
length = [i for i in range(len(force_list))]
fig = plt.figure()
plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9,
                    wspace=0.4, hspace=0.4)
plt.title("force", pad=10, fontsize=20)
ax1 = fig.add_subplot(111)
data_plot(ax1, x=length, y=l[:, 0], xlabel="step", ylabel="force_x  N", is_grid=True)

# 绘制奖励函数的图
reward_list = np.array(reward_list)
length = [i for i in range(len(reward_list))]
fig1 = plt.figure()
plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9,
                    wspace=0.4, hspace=0.4)
plt.title("force", pad=10, fontsize=20)
ax1 = fig1.add_subplot(111)
data_plot(ax1, x=length, y=reward_list, xlabel="step", ylabel="reward", is_grid=True)
plt.show()
