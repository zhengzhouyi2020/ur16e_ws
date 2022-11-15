#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/11/8
# @Author : Zzy

#################### 非线性微分跟踪器 ####################
import matplotlib.pyplot as plt
import numpy as np
from numpy import sign, sqrt


def fhan(x1, x2, u, r, h):
    d = r * h  # h为滤波因子，r为调节系数，r越大跟踪效果越好，但微分信号会增加高频噪声
    d0 = d * h  # 反之，微分信号越平滑，会产生一定的滞后
    y = x1 - u + h * x2
    a0 = sqrt(d * d + 8 * r * abs(y))
    if abs(y) <= d0:
        a = x2 + y / h
    else:
        a = x2 + 0.5 * (a0 - d) * sign(y)
    if abs(a) <= d:
        f = -r * a / d
    else:
        f = -r * sign(a)
    return f


class NTD:
    def __init__(self, T, r, h, expect_force):
        self.T = T
        self.r = r
        self.h = h
        self.x1 = 0
        self.x2 = 0
        self.expect_force = expect_force
        self.x1_list = []
        self.x2_list = []
        pass

    def calculate(self, u=None):
        if u is None:
            u = self.expect_force
        f = fhan(self.x1, self.x2, u, self.r, self.h)
        self.x1 = self.x1 + self.T * self.x2
        self.x2 = self.x2 + self.T * f
        self.x1_list.append(self.x1)
        self.x2_list.append(self.x2)

    def getResult(self, time):
        """前几秒阻抗控制的时间"""
        for _ in range(int(time/self.T)):
            self.calculate()
        return self.x1_list

    def plot_and_save(self):
        path_name = "feed_forward_force.csv"
        timestamp = [i * self.T for i in range(len(self.x1_list))]
        save_data = np.vstack([timestamp, self.x1_list]).T
        print(save_data.shape)
        np.savetxt(path_name, X=save_data, fmt="%.6f", delimiter=',')
        plt.plot(save_data[:, 0], save_data[:, 1])
        # plt.plot(save_data[:, 1], self.x2_list)
        plt.show()


if __name__ == '__main__':
    ## 让期望力在1s钟之内达到期望值
    ntd = NTD(T=0.02, r=25, h=0.25, expect_force=-20)
    for i in range(500):
        ntd.calculate()
    print(ntd.x1_list[100])
    ntd.plot_and_save()
