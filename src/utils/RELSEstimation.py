import math

import numpy as np
import matplotlib.pyplot as plt

from src.utils.RLSEstimation import RLSEstimation


class RELSEstimation:
    """
    递归最小二乘估计
    Y = ΦΘ + ξ
    计算噪声的估计值
    """

    def __init__(self, alpha, Xe, dt):
        self.alpha = alpha
        self.Xe = Xe
        self.P = np.eye(4)
        self.K = np.zeros((4, 1))
        self.pre_F = 0
        self.fai = np.zeros((4, 1))
        self.theta = np.zeros((4, 1))
        self.dt = dt  # 采样时间
        self.alpha_min = 0.9
        self.beta = 0.1
        self.e = np.array([0, 0])

    def getStiffness(self, X, F):
        Y = self.pre_F + F
        self.alpha = self.alpha_min + (1 - self.alpha_min) * math.exp(-self.e.T @ self.e * self.beta)
        G = (self.P @ self.fai) / (self.alpha + self.fai.T @ self.P @ self.fai)
        self.e = Y - self.fai.T @ self.theta
        self.theta = self.theta + G @ (Y - self.fai.T @ self.theta)
        self.P = (np.eye(4) - G @ self.fai.T) @ self.P / self.alpha
        self.pre_F = F

        # 更新fai参数
        self.fai[2] = self.fai[3]
        self.fai[3] = Y - self.fai.T @ self.theta
        self.fai[0] = self.fai[1]
        self.fai[1] = X - self.Xe

        B = self.dt * (self.theta[0] - self.theta[1]) / 4
        K = (self.theta[0] + self.theta[1]) / 2
        return np.array([B, K])


def test():
    RLS = RELSEstimation(0.95, 0.9, 0.01)
    Xe = 0.9
    B = 500
    K = 30000
    datas = []
    pose = []
    f = []
    for i in range(1000):
        X = 0.9 - 0.01 * i * 0.02
        F = K * (X - Xe) + B * 0.01
        data = RLS.getStiffness(X, F)
        datas.append(data)
        pose.append(X)
        f.append(F)
    l = [i for i in range(len(datas))]
    datas = np.array(datas)
    plt.plot(l, datas[:, 0])
    plt.plot(l, datas[:, 1])
    # plt.plot(pose,f)
    plt.show()


def main():
    plt.rcParams['font.family'] = 'Times New Roman'  # 全局字体样式
    plt.rcParams['font.size'] = 15  # 全局字体大小
    plt.rcParams['axes.linewidth'] = 1
    file_name = "../real_control/data/20221027/202210271645_m400_b7500_k500_LyaAdv_real_robot.csv"
    data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    pose_z = data[:350, 3]
    force_z = data[:350, 16]
    datas = []

    RLS = RELSEstimation(0.95, 0.259917, 0.02)
    for i in range(len(pose_z)):
        data = RLS.getStiffness(pose_z[i], force_z[i])
        datas.append(data)
    l = [i*0.02 for i in range(len(datas))]
    datas = np.array(datas)
    print(datas[-1])
    # plt.plot(l, datas[:, 0])
    plt.plot(l, datas[:, 1])
    plt.xlabel("time(s)")
    plt.ylabel("K(N/m)")
    plt.show()
    datas = datas.reshape((-1,2))
    np.savetxt("RLSEstimation.csv", X=datas, fmt="%.6f",delimiter=',')

    # plt.plot(-pose_z, -force_z)
    # plt.show()


if __name__ == '__main__':
    main()
