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


class RealEnv:
    def __init__(self, rtde_r, trajectory, expect_force=0, m=1, init_point=None):
        """
        考虑笛卡尔空间坐标系的阻抗控制
        """
        self.delta_force = 0
        self.delta_x = 0
        self.data_list = []
        self.control_step = 0.02  # 控制周期时间
        self.actual_pose = None  # 实际位姿
        self.actual_force = None  # 实际接触力
        self.actual_q = None  # 实际角位移

        self.index = 0

        self.rtde_r = rtde_r  # 信息接收

        self.pre_d_p = None  # 上一时刻位置指令
        self.pre_p = None  # 上一时刻速度指令

        self.m = 200  # 惯性系数
        self.b = 10000  # 阻尼参数
        self.k = 10  # 刚度系数

        self.ur16e_kinematics = Kinematic()  # 正逆运动学
        self.init_point = init_point  # 初始运动控制点

        self.trajectory = trajectory  # 参考轨迹
        self.expect_force = expect_force  # 参考力 1  向量
        self.force_client = rospy.ServiceProxy('/server_for_force', ForceAndTorque)  # 通过服务得到六维力数据
        self.robot_command = robot_client()

    def reset(self):
        """
        """
        if self.init_point is None:
            qpos = [0, -1.57, 1.57, -1.57, -1.57, 0.0]
        else:
            qpos = self.init_point
        self.robot_command.move_to_joint(qpos, 10)  # 关节位移控制，
        self.index = 0

    def step(self, action):
        self.admittance(action)  # 更新了当前位置，当前速度，当前力

        time_stamp, self.actual_force = self._get_actual_force()
        self.delta_force = abs(self.actual_force[2] - self.expect_force)

        observation = self._get_obs()
        reward = self._get_reward()
        information = self._get_info()
        done = self._get_done()
        return observation, reward, done, information

    def admittance(self, action):
        self.m = 500 + action * 300
        self.b = 80000 + action * 30000
        desire_pose = self.trajectory[self.index + 1]  # 期望轨迹数据
        position, orientation = desire_pose[:3], desire_pose[3:7]  # TODO 数据格式注意

        time_stamp, self.actual_force = self._get_actual_force()  # 得到真实的接触力
        delta_F_tool = self.actual_force[2] - self.expect_force  # 与期望力偏差，只考虑法向力的作用
        # TODO 姿态阻抗有点难度，主要是力矩变换成基坐标系的难度
        # 阻抗控制核心计算控制率
        delta_dd_p = 1 / self.m * (delta_F_tool - self.b * self.pre_d_p - self.k * self.pre_p)
        delta_d_p = self.pre_d_p + delta_dd_p * self.control_step
        delta_p = self.pre_p + delta_d_p * self.control_step

        self.pre_p = delta_p
        self.pre_d_p = delta_d_p  # 供新一轮的迭代

        self.delta_x = delta_p

        transform = pose2mat(desire_pose)
        next_contact = np.dot(transform, [0, 0, delta_p, 1])  # 只在Z轴方向进行移动
        pose = np.array(
            [next_contact[0], next_contact[1], next_contact[2], orientation[0], orientation[1], orientation[2],
             orientation[3]])
        joint_angles = self.ik(pose)
        force_pose = np.hstack([time_stamp, pose, joint_angles, self.actual_force])

        self.data_list.append(force_pose)

        # 用来安全性判断
        sum_angles = 0
        for i in range(6):
            sum_angles = sum_angles + math.fabs(joint_angles[i] - self.actual_q[i])
        if math.fabs(self.actual_force[2]) > 50 or sum_angles > math.pi / 4 or math.fabs(
                delta_p) > 0.008:
            print("angle:{}".format(sum_angles))
            print("position:{}".format(delta_p))
            print("force:{}".format(math.fabs(self.actual_force[2])))
            print("please check your parameter!")
            exit(0)

        self.robot_command.move_to_joint(joint_angles, self.control_step)
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
        return np.array([self.delta_x, self.delta_force], dtype=np.float32)

    def _get_reward(self):
        if self.delta_force > 10:
            reward = -5
        else:
            reward = 1/(0.1 + 0.1 * self.delta_force)
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

    def _get_actual_force(self):
        self.actual_q = self.rtde_r.getActualQ()  # 更新角度
        transform = self.ur16e_kinematics.FKine(self.actual_q)
        self.actual_pose = mat2pose(transform)  # 得到实际位置

        HexForce = self.force_client.call()  # 更新接触力
        time_stamp = HexForce.timeStamp
        return time_stamp, GravityCompensation(transform[0:3, 0:3],
                                               np.array(HexForce.forceData))
