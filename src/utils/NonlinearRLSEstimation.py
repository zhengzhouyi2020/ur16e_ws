import math

import numpy as np
import matplotlib.pyplot as plt


class NonlinearRLSEstimation:
    """
    递归最小二乘估计
    Y = ΦΘ
    """

    def __init__(self, alpha, Xe, dt):
        self.alpha = alpha
        self.Xe = Xe
        self.P = np.eye(2)
        self.K = np.zeros((2, 1))
        self.pre_F = 0
        self.fai = np.zeros((2, 1))
        self.theta = np.zeros((2, 1))
        self.dt = dt  # 采样时间

    def getStiffness(self, X, F):
        Xk = X - self.Xe
        # 更新fai参数
        self.fai[0] = 1
        self.fai[1] = Xk_dot
        self.fai[2] = math.log(Xk)

        G = (self.P @ self.fai) / (self.alpha + self.fai.T @ self.P @ self.fai)
        self.theta = self.theta + G @ (Y - self.fai.T @ self.theta)
        self.P = (np.eye(2) - G @ self.fai.T) @ self.P / self.alpha

        self.pre_F = F
        B = self.dt * (self.theta[0] - self.theta[1]) / 4
        K = (self.theta[0] + self.theta[1]) / 2
        return np.array([B, K])


def test():
    RLS = RLSEstimation(0.95, 0.9, 0.01)
    Xe = 0.9
    B = 500
    K = 30000
    datas = []
    pose = []
    f = []
    for i in range(1000):
        X = 0.9 - 0.01 * i * 0.02
        F = K * (X - Xe)**3 + B * 0.01**3
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
    file_name = "../real_control/data/202210141147_m400_b10000_k400_real_robot_20N.csv"
    data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    pose_z = data[:, 3]
    force_z = data[:, 16]
    datas = []
    print(len(pose_z))
    RLS = RlSEstimation(0.8, 0.323425642242793, 0.02)
    for i in range(len(pose_z)):
        data = RLS.getStiffness(pose_z[i], force_z[i])
        datas.append(data)
    l = [i for i in range(len(datas))]
    datas = np.array(datas)
    plt.plot(l, datas[:, 0])
    # plt.plot(l, datas[:, 1])
    plt.show()
    # plt.plot(pose_z, force_z)
    plt.show()



if __name__ == '__main__':
    main()
