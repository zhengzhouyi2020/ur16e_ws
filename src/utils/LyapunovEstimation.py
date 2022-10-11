#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/10/11
# @Author : Zzy


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
        self.initXe = initXe
        self.initKe = initKe
        self.Xe = 0
        self.Ke = 0
        self.Fd = Fd
        self.dt = dt

    def getTraject(self, F, X):
        """
        @param F: 传入的接触力
        @param X: 当前Z的位置
        @return: 在线生成的轨迹
        """
        f_wave = self.Ke * (X - self.Xe) - F
        self.Xe = self.initXe - self.alpha * X * f_wave * self.dt  # 估计的环境位置
        self.Ke = self.initKe + f_wave / self.Ke * (self.alpha * X * self.Xe + self.beta) * self.dt  # 估计的环境刚度
        Xd = self.Xe + self.Fd / self.Ke
        return Xd
