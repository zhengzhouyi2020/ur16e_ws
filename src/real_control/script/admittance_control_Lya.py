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

from real_control.srv import ForceAndTorque
from src.real_control.script.admittance_control_release import Admittance_control
from src.real_control.script.robot_control import robot_client
from src.real_control.script.ur16e_kinematics import Kinematic, mat2pose, get_Jacobi, axisangle2quat, pose2mat, \
    GravityCompensation

# 这个程序使用过程中无法收敛
from src.utils.LyapunovEstimation import LyapunovEstimation


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
    length = [i for i in range(len(pose_list))]
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
    file_name = "../data/202207182040_robot.csv"
    m = 400  # 质量系数
    k = 200  # 刚度系数
    b = 8000  # 阻尼系数
    rospy.init_node("control_node")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")
    data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    init_angle = data[0, 8:14]
    trajectory = data[:, 1:8]
    lya = LyapunovEstimation(10, 10, 15000, 0.5, 0.02, -15)
    admittance_control = Admittance_control(rtde_r, trajectory=trajectory, expect_force=-15, init_point=init_angle)
    while admittance_control.over is False:
        pose_Z = admittance_control.getNextPoint()[2]
        F = admittance_control.getContactForce()[2]
        Zd = lya.getTraject(F, pose_Z)
        admittance_control.setNextPoint(Zd)
        admittance_control.admittance(m=m, b=b, k=k)
    path_name = '../data/' + datetime.now().strftime('%Y%m%d%H%M') + "_m" + str(
        m) + "_b" + str(b) + "_k" + str(k) + '_real_robot.csv'
    pose_list = admittance_control.control_pose_list
    np.savetxt(path_name, X=pose_list, delimiter=',')
    plot_data(pose_list)


if __name__ == '__main__':
    main()
