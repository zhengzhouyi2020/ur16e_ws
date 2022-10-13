#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/10/13
# @Author : Zzy
import math

import numpy as np
import rospy
import rtde_receive
from datetime import datetime

from real_control.srv import ForceAndTorque
from src.real_control.script.robot_control import robot_client

# 实验要注意
from src.real_control.script.ur16e_kinematics import Kinematic, axisangle2quat, pose2mat, GravityCompensation

# 逆运动学类
ur16e_kinematics = Kinematic()
# 运动信息类
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")
# 力信息获取类
rospy.wait_for_service('/server_for_force')
force_client = rospy.ServiceProxy('/server_for_force', ForceAndTorque)


def getContactForce():
    HexForce = force_client.call()  # 更新接触力
    q = rtde_r.getActualQ()  # 更新角度
    transform = ur16e_kinematics.FKine(q)
    return GravityCompensation(transform[0:3, 0:3], np.array(HexForce.forceData))


def isLimit(pre_angle, cur_angle):
    sum_angles = 0
    for i in range(6):
        sum_angles = sum_angles + math.fabs(pre_angle[i] - cur_angle[i])
    return sum_angles < math.pi / 4 and abs(getContactForce()[2]) < 50


def ik(pose):
    """
    一般使用位置四元数控制，也可以用轴角控制来做，做的少
    """
    if len(pose) == 6:
        pose[3:] = axisangle2quat(pose[3:])
    Transform = pose2mat(pose)
    jnt_init = rtde_r.getActualQ()
    jnt_out = ur16e_kinematics.IKine(Transform, jnt_init)
    return jnt_out


def main():
    rospy.init_node("control_node")
    robot_command = robot_client()
    init_angle = []  # 初始位置的关节角，一定保证接触力刚好为零的位置
    init_pose = []  # 初始位置的位姿表示， 一定保证接触力刚好为零
    robot_command.move_to_joint(init_angle, time1=10)
    pose = init_pose
    pre_angle = init_angle
    for i in range(2500):
        pose[2] = pose[2] - 0.000002
        angle = ik(pose)
        if not isLimit(pre_angle, angle):
            break
        pre_angle = angle
        robot_command.move_to_joint(angle, time1=0.02)

    print("System quit...")


if __name__ == '__main__':
    main()
