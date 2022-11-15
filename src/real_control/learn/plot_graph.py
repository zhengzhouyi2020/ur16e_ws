#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/11/2
# @Author : Zzy
import os


def save_graph():
    print("============================================================================================")
    fig_num = 0
    figures_dir = "figs"
    if not os.path.exists(figures_dir):
        os.makedirs(figures_dir)
    fig_save_path = figures_dir + '/admittance_control_' + str(fig_num) + '.svg'

    # get number of log files in directory
    log_dir = "logs" + '/admittance_control' + '/'

    # TODO 绘制强化学习的奖励图

if __name__ == '__main__':
    save_graph()
