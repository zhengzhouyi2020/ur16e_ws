#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/6/14
# @Author : Zzy
import math
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rospy
import rtde_receive

from src.real_control.script.admittance_control_release import Admittance_control

from src.utils.LyapunovEstimation import LyapunovEstimationImprove
import os

from src.utils.NTD import NTD

save_path = '../data/' + datetime.now().strftime('%Y%m%d')
if not os.path.exists(save_path):
    os.mkdir(save_path)


def data_plot(ax, x, y, xlabel, ylabel, title="", color='r', is_grid=False):
    ax.plot(x, y, color=color, linestyle='-.')
    ax.set_title(title)
    ax.spines['right'].set_visible(False)  # 设置右侧坐标轴不可见
    ax.spines['top'].set_visible(False)  # 设置上坐标轴不可见
    ax.spines['top'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.spines['right'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.set_xlabel(xlabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离
    ax.set_ylabel(ylabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离


def plot_data(pose_list, index=14):
    """
    @param pose_list: 所有的数据
    @param index: 14 - 实际接触力 20 - 实际的测量力
    @return:
    """
    l = np.array(pose_list)
    length = [i * 0.02 for i in range(len(pose_list))]
    fig = plt.figure()
    ax1 = fig.add_subplot(231)
    data_plot(ax1, x=length, y=l[:, index + 0], xlabel="step", ylabel="force_x  N", is_grid=True)
    ax2 = fig.add_subplot(232)
    data_plot(ax2, length, l[:, index + 1], "step", "force_y  N")
    ax3 = fig.add_subplot(233)
    data_plot(ax3, length, l[:, index + 2], "step", "force_z  N")
    ax4 = fig.add_subplot(234)
    data_plot(ax4, length, l[:, index + 3], "step", "torque_x  mN")
    ax5 = fig.add_subplot(235)
    data_plot(ax5, length, l[:, index + 4], "step", "torque_y  mN")
    ax6 = fig.add_subplot(236)
    data_plot(ax6, length, l[:, index + 5], "step", "torque_z  mN")
    plt.show()


####
# 其实可以设置TCP的偏置量，
# 然后直接从RTDE中得到末端的位置和速度
####

# 数据记录格式
# 0-时间
# 1-8 位姿
# 8-14 关节角
# 14-20 实际接触力
# 20-26 测量力

def main():
    # file_name = "../data/20221021/202210211016_robot.csv"
    # data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    # init_angle = data[0, 8:14]
    m = 400  # 质量系数
    k = 500  # 刚度系数
    b = 7500  # 阻尼系数
    rospy.init_node("control_node")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")

    file_2 = "trajectory/trajectory.csv"
    # trajectory = data[:, 1:8]
    #
    file_2 = "trajectory2.csv"
    init_angle = [0.26145, -1.5196, -1.98123, -1.22225, 1.57067, -0.554393]

    temp = np.loadtxt(open(file_2, "rb"), delimiter=",", skiprows=1)
    trajectory = temp

    #### 生成前几秒的期望力 ####
    ntd = NTD(T=0.02, r=25, h=0.25, expect_force=-20)
    feed_force = ntd.getResult(2)  # 得到前反馈力的表达式

    ## 李雅普诺夫横磨的好参数 0.4 0.4 0.1
    lya = LyapunovEstimationImprove(alpha=1.8, beta=1.8, gamma=0.6, initKe=7750, initXe=0.260950, dt=0.02, Fd=-20)
    admittance_control = Admittance_control(rtde_r, trajectory=trajectory, expect_force=-20, init_point=init_angle)

    i = 0
    while admittance_control.over is False:
        # 设置变化的期望力，以免引起冲击振荡
        expect_force = -20
        if i < len(feed_force):
            admittance_control.setExpectForce(feed_force[i])
            expect_force = feed_force[i]
            i = i + 1

        admittance_control.update()

        F = admittance_control.actual_force[2]
        pose_Z = admittance_control.actual_pose[2]
        if admittance_control.index <= 350 or admittance_control.index >= 540:
            v = 0
        else:
            v = -0.00141
        Zd = lya.getTraject(F, pose_Z, expect_force, v)
        admittance_control.setNextPoint(Zd)
        admittance_control.admittance(m=m, b=b, k=k)
    lya.finish()
    path_name = '../data/' + datetime.now().strftime('%Y%m%d') + '/' + datetime.now().strftime(
        '%Y%m%d%H%M') + "_m" + str(
        m) + "_b" + str(b) + "_k" + str(k) + '_LyaAdv_real_robot.csv'
    pose_list = admittance_control.control_pose_list
    np.savetxt(path_name, X=pose_list, fmt="%.6f", delimiter=',')
    end_angle = [0.366, -1.67, -1.625, -1.428316981797554, 1.572, -0.358]
    admittance_control.robot_command.move_to_joint(end_angle, 5)
    plot_data(pose_list)


if __name__ == '__main__':
    main()
