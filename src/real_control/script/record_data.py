#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/6/2
# @Author : Zzy

import math
from datetime import datetime
from math import sqrt, sin, cos

import numpy as np
import rtde_receive
import rospy
import os
import csv

from real_control.srv import ForceAndTorque

from src.real_control.script.ur16e_kinematics import axisangle2quat, quat2mat, GravityCompensation, Kinematic, mat2pose

# 数据记录格式
# 0-时间
# 1-8 位姿
# 8-14 关节角
# 14-20 实际接触力
# 20-26 测量力

EPS = 1e-6
if not os.path.exists('../data/20221021'):
    os.mkdir('../data/20221021')

ur16e = Kinematic()


def main():
    rospy.init_node('record_data')
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")
    rospy.wait_for_service('/server_for_force')

    force_client = rospy.ServiceProxy('/server_for_force', ForceAndTorque)
    f = open('../data/20221021/' + datetime.now().strftime('%Y%m%d%H%M') + '_robot.csv', 'w',
             encoding='utf - 8', newline='')
    csv_writer = csv.writer(f)
    csv_writer.writerow(
        ["timestamp", "position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w",
         "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
         "ContactFX", "ContactFY", "ContactFZ", "ContactRX", "ContactRY", "ContactRZ",  # 实际力
         "HexForceX", "HexForceY", "HexForceZ", "HexTorqueX", "HexTorqueY", "HexTorqueZ"  # 测量力
         ])
    print("Record data now!")
    while True:
        HexForce = force_client.call()
        # actual_p = rtde_r.getActualTCPPose()  # 可以设置TCP的偏置
        # quat = axisangle2quat(actual_p[3:])
        # R = quat2mat(quat)
        print(HexForce)
        actual_q = rtde_r.getActualQ()
        Transform = ur16e.FKine(actual_q)
        actual_p = mat2pose(Transform)
        R = Transform[0:3, 0:3]
        actual_F = GravityCompensation(R, np.array(HexForce.forceData))
        csv_writer.writerow(
            (HexForce.timeStamp, actual_p[0], actual_p[1], actual_p[2], actual_p[3], actual_p[4], actual_p[5],actual_p[6],
             actual_q[0], actual_q[1], actual_q[2], actual_q[3], actual_q[4], actual_q[5],
             actual_F[0], actual_F[1], actual_F[2],actual_F[3], actual_F[4], actual_F[5],
             HexForce.forceData[0], HexForce.forceData[1], HexForce.forceData[2], HexForce.forceData[3], HexForce.forceData[4], HexForce.forceData[5],
             ))


if __name__ == '__main__':
    main()
