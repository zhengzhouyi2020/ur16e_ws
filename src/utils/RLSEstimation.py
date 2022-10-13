import numpy as np
import matplotlib.pyplot as plt

class RLSEstimation:
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
        Y = self.pre_F + F
        # 更新fai参数
        self.fai[0] = self.fai[1]
        self.fai[1] = X - self.Xe
        G = (self.P @ self.fai) / (self.alpha + self.fai.T @ self.P @ self.fai)
        self.theta = self.theta + G @ (Y - self.fai.T @ self.theta)
        self.P = (np.eye(2) - G @ self.fai.T) @ self.P / self.alpha
        self.pre_F = F
        B = self.dt * (self.theta[0] - self.theta[1]) / 4
        K = (self.theta[0] + self.theta[1]) / 2
        return np.array([B, K])


if __name__ == '__main__':
    RLS = RLSEstimation(0.95, 0.9, 0.01)
    Xe = 0.9
    B = 500
    K = 3000
    datas = []
    for i in range(1000):
        X = 0.9 - 0.01 * i*0.02
        F = K * (X - Xe) + B * 0.01
        data = RLS.getStiffness(X, F)
        datas.append(data)
    l = [i  for i in range(len(datas))]
    datas = np.array(datas)
    plt.plot(l,datas[:,0])
    plt.plot(l, datas[:, 1])
    plt.show()