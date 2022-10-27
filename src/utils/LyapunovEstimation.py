#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/10/11
# @Author : Zzy
from datetime import datetime

import numpy as np


class LyapunovEstimation:

    def __init__(self, alpha, beta, initKe, initXe, dt, Fd):
        """
        @param alpha:  系数1
        @param beta: 系数2
        @param initKe: 初始迭代刚度
        @param initXe: 初始位置
        @param dt: 控制周期
        @param Fd: 期望力
        """
        self.alpha = alpha
        self.beta = beta
        self.Xe = initXe
        self.Ke = initKe
        self.Fd = Fd
        self.dt = dt
        self.Xe_list = []
        self.Ke_list = []
        self.Xd_list = []

    def getTraject(self, F, X):
        """
        @param F: 传入的接触力
        @param X: 当前Z的位置
        @return: 在线生成的轨迹
        """

        f_wave = self.Ke * (X - self.Xe) - F
        self.Ke = self.Ke - self.alpha * X * f_wave * self.dt  # 估计的环境位置
        self.Xe = self.Xe + f_wave / self.Ke * (self.alpha * X * self.Xe + self.beta) * self.dt  # 估计的环境刚度

        Xd = self.Xe + self.Fd / self.Ke

        self.Xe_list.append(self.Xe)
        self.Ke_list.append(self.Ke)
        self.Xd_list.append(Xd)
        return Xd


    def finish(self):
        data = np.vstack([self.Xe_list,self.Ke_list,self.Xd_list]).T
        path_name = '../data/20221027/' + datetime.now().strftime('%Y%m%d%H%M') + '_Lya_estimate.csv'
        np.savetxt(path_name, X=data, fmt="%.6f",delimiter=',')


class LyapunovEstimationImprove:

    def __init__(self, alpha, beta, gamma, initKe, initXe, dt, Fd):
        """
        @param alpha:  系数1
        @param beta: 系数2
        @param initKe: 初始迭代刚度
        @param initXe: 初始位置
        @param dt: 控制周期
        @param Fd: 期望力
        """
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.Xe = initXe
        self.Ke = initKe
        self.Fd = Fd
        self.dt = dt
        self.Xe_list = []
        self.Ke_list = []
        self.Xd_list = []

    def getTraject(self, F, X, velocity = 0):
        """
        @param velocity: z方向的速度
        @param F: 传入的接触力
        @param X: 当前Z的位置
        @return: 在线生成的轨迹
        """
        f_wave = self.Ke * (X - self.Xe) - F
        self.Ke = self.Ke - self.alpha * X * f_wave * self.dt + self.gamma * f_wave * self.dt  # 估计的环境位置
        self.Xe = self.Xe + f_wave / self.Ke * (
                self.alpha * X * self.Xe + self.beta - self.gamma * X - self.gamma * self.Xe) * self.dt + velocity * self.dt  # 估计的环境刚度
        Xd = self.Xe + self.Fd / self.Ke
        self.Xe_list.append(self.Xe)
        self.Ke_list.append(self.Ke)
        self.Xd_list.append(Xd)
        return Xd

    def finish(self):
        data = np.vstack([self.Xe_list, self.Ke_list, self.Xd_list]).T
        path_name = '../data/20221027/' + datetime.now().strftime('%Y%m%d%H%M') + '_LyaAdv_estimate.csv'
        np.savetxt(path_name, X=data, fmt="%.6f", delimiter=',')
