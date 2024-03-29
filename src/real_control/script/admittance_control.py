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


def data_plot(ax, x, y, xlabel, ylabel, title="", color='r', is_grid=False):
    ax.plot(x, y, color=color, linestyle='-.')
    ax.set_title(title)
    ax.spines['right'].set_visible(False)  # 设置右侧坐标轴不可见
    ax.spines['top'].set_visible(False)  # 设置上坐标轴不可见
    ax.spines['top'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.spines['right'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.set_xlabel(xlabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离
    ax.set_ylabel(ylabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离


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
        self.calculateVel()
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

    def calculateVel(self):
        for i in range(len(self.trajectory)):
            if i == 0:
                self.desire_vel[i, :] = (self.trajectory[i + 1, :3] - self.trajectory[i, :3]) / 0.02
                self.desire_acc[i + 1, :] = (self.trajectory[i + 1, :3] - 2 * self.trajectory[i, :3] + self.trajectory[
                                                                                                       i,
                                                                                                       :3]) / (
                                                    0.02 * 0.02)
            elif i == len(self.trajectory) - 1:
                self.desire_vel[i, :] = (self.trajectory[i, :3] - self.trajectory[i - 1, :3]) / self.control_step
                self.desire_acc[i, :] = (self.trajectory[i, :3] - 2 * self.trajectory[i, :3] + self.trajectory[
                                                                                                   i - 1,
                                                                                                   :3]) / (self.control_step * self.control_step)
            else:
                self.desire_vel[i, :] = (self.trajectory[i + 1, :3] - self.trajectory[i - 1, :3]) / (self.control_step * 2)
                self.desire_acc[i, :] = (self.trajectory[i + 1, :3] - 2 * self.trajectory[i, :3] + self.trajectory[
                                                                                                       i - 1,
                                                                                                       :3]) / (
                                                    self.control_step * self.control_step)

    def admittance(self, m, b, k):
        self.M = m * np.identity(3)
        self.K = k * np.identity(3)
        self.B = b * np.identity(3)
        desire_pose = self.trajectory[self.index + 1]  # 期望轨迹数据
        position, orientation = desire_pose[:3], desire_pose[3:7]  # TODO 数据格式注意

        self.actual_q = self.rtde_r.getActualQ()  # 更新角度
        self.actual_qd = self.rtde_r.getActualQd()  # 更新角速度
        transform = self.ur16e_kinematics.FKine(self.actual_q)
        self.actual_pose = mat2pose(transform)  # 得到实际位置

        Jacobi = get_Jacobi(self.actual_q)  # 雅可比矩阵
        self.actual_vel = Jacobi.dot(self.actual_qd)  # 根据当前角速度计算实际速度

        HexForce = self.force_client.call()  # 更新接触力
        time_step = HexForce.timeStamp
        self.actual_force = GravityCompensation(transform[0:3, 0:3],
                                                np.array(HexForce.forceData))

        # TODO 姿态阻抗有点难度，主要是力矩变换成基坐标系的难度

        delta_F_tool = self.actual_force[2] - self.expect_force  # 与期望  力偏差
        delta_F_tool = np.array([0, 0, delta_F_tool])  # 只考虑Z方向的力

        rotateMatrix = transform[0:3, 0:3]
        delta_F_base = np.dot(rotateMatrix, delta_F_tool)  # 转换到世界坐标系

        delta_p = self.actual_pose[0:3] - self.trajectory[self.index + 1, :3]  # 位置偏差
        delta_vel = self.actual_vel[0:3] - self.desire_vel[self.index + 1, :]  # TODO 需要速度误差

        B = np.dot(self.B, delta_vel)
        K = np.dot(self.K, delta_p)

        # TODO 这里默认期望速度和期望加速度为零，进行阻抗计算
        dd_p = self.desire_acc[self.index + 1, :] + np.dot(np.linalg.inv(self.M), (delta_F_base - B - K))
        d_p = self.actual_vel[0:3] + dd_p * self.control_step
        position = self.actual_pose[0:3] + d_p * self.control_step

        boolean_position = self.actual_pose[0:3]

        self.pre_vel = d_p
        self.pre_position = position  # 供新一轮的迭代

        pose = np.hstack([position, orientation])

        joint_angles = self.ik(pose)

        force_pose = np.hstack([time_step, pose, joint_angles, self.actual_force])
        self.control_pose_list.append(force_pose)

        sums = 0
        ##  用来判断距离是否超量程
        for i in range(6):
            sums = sums + math.fabs(joint_angles[i] - self.actual_q[i])

        if math.fabs(self.actual_force[2]) > 30 or sums > math.pi / 4 or math.fabs(
                self.pre_position[2] - boolean_position[2]) > 0.005:
            print("angle:{}".format(sums))
            print("position:{}".format(self.pre_position[2] - boolean_position[2]))
            print("HEX:{}".format(HexForce))
            print("force:{}".format(math.fabs(self.actual_force[2])))
            print("please check your parameter!")
            exit(0)
        # curtime = int(datetime.now().strftime('%H%M%S%f')[:-3]) / 1000
        self.robot_command.move_to_joint(joint_angles, self.control_step)
        self.index += 1
        if self.index == len(self.trajectory) - 1:
            self.over = True

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
    file_name = "../data/202207181706_robot.csv"

    rospy.init_node("control_node")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")
    data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    init_angle = data[0, 8:14]
    trajectory = data[:, 1:8]

    admittance_control = Admittance_control(rtde_r, trajectory=trajectory, expect_force=-10, init_point=init_angle)
    m = 100
    k = 0
    ratio = 30
    # b = 2 * ratio * math.sqrt(m * k)
    b = 500
    while admittance_control.over is False:
        admittance_control.admittance(m=m, b=b, k=k)

    path_name = '../data/' + datetime.now().strftime('%Y%m%d%H%M') + '_real_robot.csv'
    pose_list = admittance_control.control_pose_list
    np.savetxt(path_name, X=pose_list, delimiter=',')

    l = np.array(pose_list)
    length = [i for i in range(len(pose_list))]
    fig = plt.figure()
    ax1 = fig.add_subplot(231)
    data_plot(ax1, x=length, y=l[:, 15], xlabel="step", ylabel="force_x  N", is_grid=True)

    ax2 = fig.add_subplot(232)
    data_plot(ax2, length, l[:, 16], "step", "force_y  N")

    ax3 = fig.add_subplot(233)
    data_plot(ax3, length, l[:, 17], "step", "force_z  N")

    ax4 = fig.add_subplot(234)
    data_plot(ax4, length, l[:, 18], "step", "force_x  N")

    ax5 = fig.add_subplot(235)
    data_plot(ax5, length, l[:, 19], "step", "force_x  N")

    ax6 = fig.add_subplot(236)
    data_plot(ax6, length, l[:, 19], "step", "force_x  N")

    plt.show()


if __name__ == '__main__':
    main()
