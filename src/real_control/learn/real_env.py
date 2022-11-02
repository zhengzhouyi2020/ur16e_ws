#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/6/5
# @Author : Zzy
import math

import numpy as np
import rospy
import rtde_control
import rtde_receive

from real_control.srv import ForceAndTorque
from src.real_control.script.robot_control import robot_client
from src.real_control.script.ur16e_kinematics import Kinematic, mat2pose, get_Jacobi, axisangle2quat, pose2mat, \
    GravityCompensation


# 数据记录格式 0-时间 1-8 位姿 8-14 关节角 14-20 实际接触力 20-26 测量力


class RealEnv:
    def __init__(self, rtde_r, trajectory, init_point, expect_force=-15, m=400, b=8000, k=800):
        """
        考虑笛卡尔空间坐标系的阻抗控制
        """

        self.control_step = 0.02  # 控制周期时间

        self.time_step = 0  # 数据采集时间
        self.actual_pose = None  # 实际位姿
        self.actual_force = None  # 实际接触力
        self.actual_q = None  # 实际角位移
        self.actual_qd = None  # 实际速度
        self.actual_vel = None  # 实际速度

        self.desire_vel = np.zeros([len(trajectory), 3])  # 期望速度， 为李雅普诺夫方法作为基础

        self.delta_force = 0  # 接触力的差值
        self.delta_x = 0  # 每次的调整量

        self.index = 0

        self.rtde_r = rtde_r  # 信息接收

        self.pre_p = None  # 上一时刻位置指令
        self.pre_d_p = None  # 上一时刻速度指令

        self.m = m  # 惯性系数
        self.b = b  # 阻尼参数
        self.k = k  # 刚度系数

        self.ur16e_kinematics = Kinematic()  # 正逆运动学
        self.init_point = init_point  # 初始运动控制点

        self.trajectory = trajectory  # 参考轨迹
        self.expect_force = expect_force  # 参考力 1  向量
        self.force_client = rospy.ServiceProxy('/server_for_force', ForceAndTorque)  # 通过服务得到六维力数据
        self.robot_command = robot_client()

        self.data_list = []  # 保存所有数据

        self.reset()
        self.calculateVel()
        print("Init success now")

    def calculateVel(self):
        # 计算工件轮廓的速度，只计算Z方向的速度
        for i in range(len(self.trajectory)):
            if i == 0:
                self.desire_vel[i] = (self.trajectory[i + 1, 2] - self.trajectory[i, 2]) / 0.02
            elif i == len(self.trajectory) - 1:
                self.desire_vel[i] = (self.trajectory[i, 2] - self.trajectory[i - 1, 2]) / self.control_step
            else:
                self.desire_vel[i] = (self.trajectory[i + 1, 2] - self.trajectory[i - 1, 2]) / (self.control_step * 2)

    def reset(self):
        self.robot_command.move_to_joint(self.init_point, 5)  # 关节位移控制，回到初始位置
        self.update()  # 在训练开始前先进行数据更新
        self.index = 0

    def step(self, action):

        self.admittance(action)  # 更新了当前位置，当前速度，当前力
        self.update()  # 需要更新力、位置等信息
        self.delta_force = abs(self.actual_force[2] - self.expect_force)

        observation = self._get_obs()
        reward = self._get_reward()

        information = self._get_info()
        done = self._get_done()
        return observation, reward, done, information

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

    # 需要考虑实时更新数据和进行控制的先后顺序，获取数据有时间限制
    def admittance(self, action):
        # TODO 可能在原有的基础上添加会更好点 k = k + delta_k b = b + delta_b

        self.m = 500 + action * 300
        self.b = 80000 + action * 30000

        desire_pose = self.trajectory[self.index + 1]  # 期望轨迹数据
        orientation = desire_pose[3:7]  # 方向数据

        delta_F_tool = self.actual_force[2] - self.expect_force  # 与期望力偏差，只考虑法向力的作用

        # 阻抗控制核心计算控制律
        # 默认期望速度和期望加速度为0，在法向上的打磨的速度比较慢
        delta_dd_p = 1 / self.m * (delta_F_tool - self.b * self.pre_d_p - self.k * self.pre_p)
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
        self.data_list.append(force_pose)

        # 用安全性的控制，如果接触力或者扭转关节太大，就回到初始控制点
        sum_angles = 0
        for i in range(6):
            sum_angles = sum_angles + math.fabs(joint_angles[i] - self.actual_q[i])
        if math.fabs(self.actual_force[2]) > 50 or sum_angles > math.pi / 4 or math.fabs(
                delta_p) > 0.008:
            print("angle:{}".format(sum_angles))
            print("position:{}".format(delta_p))
            print("force:{}".format(math.fabs(self.actual_force[2])))
            print("please check your parameter!")
            self.robot_command.move_to_joint(self.init_point, 10)  # 关节位移控制，回到初始位置，防止出现危险
            exit(0)

        self.robot_command.move_to_joint(joint_angles, self.control_step, wait=False)
        self.index += 1

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

    def _get_obs(self):
        # TODO
        """
        状态空间
        @return: 
        """
        return np.array([self.delta_x, self.delta_force], dtype=np.float32)

    def _get_reward(self):
        #  TODO 奖励函数的设置
        if self.delta_force > 10:
            reward = -5
        else:
            reward = 1 / (0.1 + 0.1 * self.delta_force)
        return reward

    def _get_info(self):
        info = dict()
        info['qpos'] = self.actual_q
        info['force'] = self.actual_force
        info['m'] = self.m
        info['b'] = self.b
        return info

    def _get_done(self):
        if self.index == len(self.trajectory) - 1 or self.delta_force > 30:
            return True
        return False
