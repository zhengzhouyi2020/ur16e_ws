import numpy as np
import matplotlib.pyplot as plt



plt.rcParams['font.family'] = 'Times New Roman'  # 全局字体样式
plt.rcParams['font.size'] = 15  # 全局字体大小
plt.rcParams['axes.linewidth'] = 1


def data_plot(ax, x, y, xlabel, ylabel, title="", color='r', is_grid=False):
    ax.plot(x, y, color=color, linestyle='-')
    ax.set_title(title)
    ax.spines['right'].set_visible(False)  # 设置右侧坐标轴不可见
    ax.spines['top'].set_visible(False)  # 设置上坐标轴不可见
    ax.spines['top'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.spines['right'].set_linewidth(0.6)  # 设置坐标轴线宽
    ax.set_xlabel(xlabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离
    ax.set_ylabel(ylabel, fontsize=15, labelpad=5)  # 设置横轴字体和距离坐标轴的距离


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


def test():
    RLS = RLSEstimation(0.1, 0.9, 0.01)
    Xe = 0.9
    B = 500
    K = 30000
    datas = []
    pose = []
    f = []
    for i in range(1000):
        X = 0.9 - 0.01 * i * 0.02
        F = K * (X - Xe) ** 3 + B * 0.01 ** 3
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
    file_name = "../real_control/data/202210141019_robot.csv"
    data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    pose_z = data[800:, 3]
    force_z = data[800:, 16]
    datas = []
    print(len(pose_z))
    RLS = RLSEstimation(0.9, 0.323425642242793, 0.02)
    for i in range(len(pose_z)):
        data = RLS.getStiffness(pose_z[i], force_z[i])
        datas.append(data)
    l = [i * 0.02 for i in range(len(datas))]
    datas = np.array(datas)
    # plt.plot(l, datas[:, 0])
    # plt.plot(l, datas[:, 1])
    fig = plt.figure()
    # ax1 = fig.add_subplot(111)
    # data_plot(ax1, l, datas[:, 0], "time(s)", "result")
    #
    # ax2 = fig.add_subplot(111)
    # data_plot(ax2, l, datas[:, 1], "time(s)", "stiffness", title="20N constant")
    # plt.show()
    ax2 = fig.add_subplot(111)
    data_plot(ax2, -pose_z, -force_z, "position(m)", "force(N)")
    plt.show()


if __name__ == '__main__':
    main()
