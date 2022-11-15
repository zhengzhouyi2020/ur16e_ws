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
from src.real_control.script.robot_control import robot_client
from src.real_control.script.ur16e_kinematics import Kinematic, mat2pose, get_Jacobi, axisangle2quat, pose2mat, \
    GravityCompensation

import os

from src.utils.NTD import NTD

save_path = '../data/' + datetime.now().strftime('%Y%m%d')
if not os.path.exists(save_path):
    os.mkdir(save_path)


# 这个程序使用过程中无法收敛
def data_plot(ax, x, y, xlabel, ylabel, title="", color='r', is_grid=False):
    ax.plot(x, y, color=color, linestyle='-')
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

class Admittance_control:
    def __init__(self, rtde_r, trajectory, expect_force=0, m=1, init_point=None):
        """
        考虑笛卡尔空间坐标系的阻抗控制
        """
        self.time_step = 0
        self.pre_d_p = 0
        self.pre_p = 0
        self.over = False
        self.control_step = 0.02  # 控制周期时间
        self.actual_pose = None  # 实际位姿
        self.actual_force = None  # 世界接触力
        self.actual_q = None  # 实际角位移
        self.actual_qd = None  # 实际速度
        self.actual_vel = None  # 实际速度

        self.index = 0

        self.rtde_r = rtde_r  # 信息接收

        self.pre_position = None  # 上一时刻位置指令
        self.pre_vel = None  # 上一时刻速度指令

        self.m = m  # 惯性系数

        self.M = None
        self.K = None  # 刚度系数
        self.B = None  # 阻尼系数

        self.ur16e_kinematics = Kinematic()  # 正逆运动学
        self.init_point = init_point  # 初始运动控制点

        self.trajectory = trajectory  # 参考轨迹
        self.desire_vel = np.zeros([len(trajectory), 3])
        self.desire_acc = np.zeros([len(trajectory), 3])
        self.expect_force = expect_force  # 参考力
        rospy.wait_for_service('/server_for_force')
        self.force_client = rospy.ServiceProxy('/server_for_force', ForceAndTorque)  # 通过服务得到六维力数据
        self.robot_command = robot_client()

        self.control_pose_list = []

        self.reset()
        print("Init success now")

    def reset(self):
        """
        """
        if self.init_point is None:
            qpos = [1.57, -1.57, -1.57, -1.57, 1.57, 0.0]
        else:
            qpos = self.init_point
        self.pre_position = self.trajectory[0, :3]
        self.pre_vel = [0, 0, 0]
        self.robot_command.move_to_joint(qpos, 5)  # 关节位移控制，

    def update(self):

        # 先更新力，再获取关节位置
        HexForce = self.force_client.call()  # 更新接触力
        self.time_step = HexForce.timeStamp

        self.actual_q = self.rtde_r.getActualQ()  # 更新角度
        self.actual_qd = self.rtde_r.getActualQd()  # 更新角速度
        transform = self.ur16e_kinematics.FKine(self.actual_q)
        self.actual_pose = mat2pose(transform)  # 得到实际位置

        Jacobi = get_Jacobi(self.actual_q)  # 雅可比矩阵
        self.actual_vel = Jacobi.dot(self.actual_qd)  # 根据当前角速度计算实际速度

        self.actual_force = GravityCompensation(transform[0:3, 0:3],
                                                np.array(HexForce.forceData))

    # 在执行admittance前必须先执行update()程序
    def admittance(self, m, b, k):
        desire_pose = self.trajectory[self.index + 1]  # 期望轨迹数据

        position, orientation = desire_pose[:3], desire_pose[3:7]  # TODO 数据格式注意

        # TODO 姿态阻抗有点难度，主要是力矩变换成基坐标系的难度
        # 直接在Z轴上进行补偿
        delta_F_tool = self.actual_force[2] - self.expect_force  # 与期望力偏差，只考虑法向力的作用
        # TODO 这里默认期望速度和期望加速度为零，进行阻抗计算
        delta_dd_p = 1 / m * (delta_F_tool - b * self.pre_d_p - k * self.pre_p)
        delta_d_p = self.pre_d_p + delta_dd_p * self.control_step
        delta_p = self.pre_p + delta_d_p * self.control_step

        self.pre_p = delta_p
        self.pre_d_p = delta_d_p  # 供新一轮的迭代

        transform = pose2mat(self.trajectory[self.index + 1])

        next_contact = np.dot(transform, [0, 0, delta_p, 1])  # 只在Z轴方向进行移动
        pose = np.array(
            [next_contact[0], next_contact[1], next_contact[2], orientation[0], orientation[1], orientation[2],
             orientation[3]])

        joint_angles = self.ik(pose)

        force_pose = np.hstack([self.time_step, pose, joint_angles, self.actual_force])
        self.control_pose_list.append(force_pose)

        sum_angles = 0
        # 用来判断距离是否超量程
        for i in range(6):
            sum_angles = sum_angles + math.fabs(joint_angles[i] - self.actual_q[i])

        if math.fabs(self.actual_force[2]) > 50 or sum_angles > math.pi / 4 or math.fabs(
                delta_p) > 0.006:
            print("angle:{}".format(sum_angles))
            print("position:{}".format(delta_p))
            print("force:{}".format(math.fabs(self.actual_force[2])))
            print("please check your parameter!")
            print(self.index)
            exit(0)
        self.robot_command.move_to_joint(joint_angles, self.control_step, wait=False)
        self.index += 1
        if self.index == len(self.trajectory) - 1:
            self.over = True

    def getActualPose(self):
        q = self.rtde_r.getActualQ()  # 更新角度
        transform = self.ur16e_kinematics.FKine(q)
        return mat2pose(transform)  # 得到实际位置

    def getNextPoint(self):
        """
        获取下一点运动的数据
        @return:
        """
        return self.trajectory[self.index + 1]  # 期望轨迹数据

    def setNextPoint(self, Zd):
        """
        @param Zd: 设置下一点Z方向的位置
        @return:
        """
        self.trajectory[self.index + 1][2] = Zd

    def setExpectForce(self, expectForce):
        """设置可变的期望力"""
        self.expect_force = expectForce

    def getActualVel(self):
        qd = self.rtde_r.getActualQd()
        q = self.rtde_r.getActualQ()  # 更新角度
        Jacobi = get_Jacobi(q)  # 雅可比矩阵
        return Jacobi.dot(qd)  # 根据当前角速度计算实际速度

    def ik(self, pose):
        """
        一般使用位置四元数控制，也可以用轴角控制来做，做的少
        """
        if len(pose) == 6:
            pose[3:] = axisangle2quat(pose[3:])
        Transform = pose2mat(pose)
        jnt_init = self.actual_q
        jnt_out = self.ur16e_kinematics.IKine(Transform, jnt_init)
        return jnt_out


def main():
    file_name = "../data/20221021/202210211016_robot.csv"

    rospy.init_node("control_node")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")
    data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    # file_2 = "trajectory/trajectory.csv"
    # init_angle = data[0, 8:14]
    # trajectory = data[:, 1:8]
    #
    file_2 = "trajectory2.csv"
    init_angle = [0.26145,-1.5196,-1.98123,-1.22225,1.57067,-0.554393]

    temp = np.loadtxt(open(file_2, "rb"), delimiter=",", skiprows=1)
    trajectory = temp

    #### 生成前几秒的期望力 ####
    ntd = NTD(T=0.02, r=100, h=0.1, expect_force=-20)
    feed_force = ntd.getResult(2)  # 得到前反馈力的表达式

    admittance_control = Admittance_control(rtde_r, trajectory=trajectory, expect_force=-20, init_point=init_angle)
    m = 400
    k = 500
    ## k = 2000时开始出现稳态误差
    ratio = 30
    # b = 2 * ratio * math.sqrt(m * k)
    b = 7500
    # b = 15000 20000 稳定时间长，控制效果不佳
    # b = 8000 10000 效果不错，
    # b = 5000 超调量有点大

    # 最优参数 800 40000
    i = 0
    while admittance_control.over is False:
        # 设置变化的期望力，以免引起冲击振荡
        if i < len(feed_force):
            admittance_control.setExpectForce(feed_force[i])
            i = i + 1
        admittance_control.update()
        admittance_control.admittance(m=m, b=b, k=k)

    path_name = '../data/' + datetime.now().strftime('%Y%m%d') + '/' + datetime.now().strftime(
        '%Y%m%d%H%M') + "_m" + str(
        m) + "_b" + str(b) + "_k" + str(k) + '_real_robot.csv'
    pose_list = admittance_control.control_pose_list
    np.savetxt(path_name, X=pose_list, fmt="%.6f", delimiter=',')
    end_angle = [0.366, -1.67, -1.625, -1.428316981797554, 1.572, -0.358]
    admittance_control.robot_command.move_to_joint(end_angle, 5)
    plot_data(pose_list)


if __name__ == '__main__':
    main()
