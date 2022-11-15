#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/6/6
# @Author : Zzy

import argparse
import os.path
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rospy
import torch

from torch.utils.tensorboard import SummaryWriter
from src.real_control.learn.PPO_continuous import Memory, PPO
from src.real_control.learn.real_env import RealEnv

import rtde_receive

# 数据记录格式 0-时间 1-8 位姿 8-14 关节角 14-20 实际接触力 20-26 测量力
# episode 训练的批次    max_timesteps 一个批次最大训练次数 update_timesteps
from src.utils.NTD import NTD

parser = argparse.ArgumentParser(description='PyTorch PPO持续控制')
parser.add_argument('--gpus', default=1, type=int, help='gpu的数量')
parser.add_argument('--solved_reward', type=float, default=10000, help='当奖励值大于其，则训练停止')
parser.add_argument('--print_interval', type=int, default=1, help='打印批次的结果')
parser.add_argument('--save_interval', type=int, default=1, help='保存批次的模型结果')
parser.add_argument('--max_episodes', type=int, default=20, help='最大批次，不能太大，否则训练不完')
parser.add_argument('--max_timesteps', type=int, default=720, help='最大时间步长，这里应该是轨迹点的个数')
parser.add_argument('--update_timesteps', type=int, default=64, help='更新策略的步长')
parser.add_argument('--action_std', type=float, default=0.5,
                    help='多维正态分布的action标准差 (Multivariate Normal)')
parser.add_argument('--action_std_decay_rate', type=float, default=0.05,
                    help='linearly decay action_std(Multivariate Normal)')
parser.add_argument('--min_action_std', type=float, default=0.1,
                    help='minimum action_std(Multivariate Normal)')
parser.add_argument('--K_epochs', type=int, default=32,
                    help='update the policy for how long time everytime')  # 存疑，计算K_epochs 步后的奖励吗
parser.add_argument('--eps_clip', type=float, default=0.2, help=' p/q 裁减率')
parser.add_argument('--gamma', type=float, default=0.9, help='discount factor')
parser.add_argument('--lr', type=float, default=0.0003)
parser.add_argument('--seed', type=int, default=123, help='可变的随机种子')
parser.add_argument('--ckpt_folder', default='./checkpoints', help='模型保存的文件夹位置')
parser.add_argument('--tb', default=True, action='store_true', help='是否使用可视化')
parser.add_argument('--log_folder', default='./logs', help='日志保存文件夹的位置')
parser.add_argument('--mode', default='test', help='选择test还是train')
parser.add_argument('--restore', default=True, action='store_true', help='是否加载已有模型继续训练')
opt = parser.parse_args()

force_list = []
reward_list = []


def train(env, state_dim, action_dim, solved_reward,
          max_episodes, max_timesteps, update_timestep, action_std, action_std_decay_rate, min_action_std, K_epochs,
          eps_clip,
          gamma, lr, betas, ckpt_folder, log_folder, restore, tb=False, print_interval=1, save_interval=1):
    ckpt = ckpt_folder + '/' + "2212_admittance_control.pth"
    if restore:
        print('Load checkpoint from {}'.format(ckpt))

    memory = Memory()

    ppo = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip, restore=restore, ckpt=ckpt)

    running_reward, avg_length, time_step = 0, 0, 0
    current_num_files = next(os.walk(log_folder))[2]
    run_num = len(current_num_files)
    log_f_name = log_folder + "/admittance_control_log_" + str(run_num) + ".csv"  # 用来保存文件

    # logging file
    log_f = open(log_f_name, "w+")
    log_f.write('episode,timestep,reward,force\n')

    #### 添加抑制冲击振荡的算法
    ntd = NTD(T=0.02, r=100, h=0.1, expect_force=-20)
    feed_force = ntd.getResult(time=2)
    # training loop
    for i_episode in range(1, max_episodes + 1):
        state = env.reset()
        force_list.clear()
        for j in range(len(feed_force)):
            env.setExpectForce(feed_force[j])
            env.admittance([0, 0])
            env.update()
            force_list.append(env.actual_force[2])
        t = 0
        for t in range(max_timesteps):
            time_step += 1
            action = ppo.select_action(state, memory)
            state, reward, done, info = env.step(action)
            memory.rewards.append(reward)
            memory.is_terminals.append(done)
            force_list.append(info['force'][2])

            if time_step % update_timestep == 0:
                ppo.update(memory)
                memory.clear_memory()
                time_step = 0

            running_reward += reward
            if done:
                break
        avg_length += t
        # 保存模型文件
        reward_list.append(running_reward)
        if running_reward > (print_interval * solved_reward) or i_episode % save_interval == 0:
            ppo.decay_action_std(action_std_decay_rate, min_action_std)
            torch.save(ppo.policy.state_dict(), ckpt_folder + "/" + datetime.now().strftime('%H%M') +
                       '_admittance_control.pth')
            print('Save a checkpoint!')
        # 计算一个打印时间的各奖励和
        if i_episode % print_interval == 0:
            avg_length = int(avg_length / print_interval)
            running_reward = int((running_reward / print_interval))
            print('Episode {} \t Avg length: {} \t Avg reward: {}'.format(i_episode, avg_length, running_reward))

            # 记录价值
            log_f.write('{},{},{}\n'.format(i_episode, time_step, running_reward))
            log_f.flush()

            if tb:
                writer.add_scalar('scalar/reward', running_reward, i_episode)
                writer.add_scalar('scalar/length', avg_length, i_episode)
            running_reward, avg_length = 0, 0
        end_angle = [0.366, -1.67, -1.625, -1.428316981797554, 1.572, -0.358]  # 结束的时候返回安全区域
        env.robot_command.move_to_joint(end_angle, 3)
    log_f.close()


def test(env, state_dim, action_dim, action_std, K_epochs, eps_clip, gamma, lr, betas, ckpt_folder,
         test_episodes):
    # 直接加载训练好的模型
    ckpt = ckpt_folder + '/2224_admittance_control.pth'
    print('Load checkpoint from {}'.format(ckpt))
    memory = Memory()
    ppo = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip, restore=True, ckpt=ckpt)

    episode_reward, time_step = 0, 0
    avg_episode_reward, avg_length = 0, 0


    #### 添加抑制冲击振荡的算法
    ntd = NTD(T=0.02, r=100, h=0.1, expect_force=-20)
    feed_force = ntd.getResult(time=2)

    # test
    for i_episode in range(1, test_episodes + 1):
        state = env.reset()
        force_list.clear()
        for i in range(len(feed_force)):
            env.setExpectForce(feed_force[i])
            env.admittance([0, 0])
            env.update()
            force_list.append(env.actual_force[2])
        while True:
            time_step += 1
            action = ppo.select_action(state, memory)
            state, reward, done, info = env.step(action)
            episode_reward += reward
            force_list.append(info['force'][2])

            if done:
                print('Episode {} \t Length: {} \t Reward: {}'.format(i_episode, time_step, episode_reward))
                avg_episode_reward += episode_reward
                avg_length += time_step
                memory.clear_memory()
                time_step, episode_reward = 0, 0
                break
    print('Test {} episodes DONE!'.format(test_episodes))
    print('Avg episode reward: {} | Avg length: {}'.format(avg_episode_reward / test_episodes,
                                                           avg_length / test_episodes))


if __name__ == '__main__':
    rospy.init_node("train_real_robot")

    # trajectory = np.loadtxt("../data/20220415_151254.csv", delimiter=',', skiprows=1)
    # initial_point = trajectory[0, 7:13]  # 设置初始的运动点, 关节角保证初始位置的准确性
    file_2 = "trajectory2.csv"
    init_angle = [0.26145, -1.5196, -1.98123, -1.22225, 1.57067, -0.554393]
    temp = np.loadtxt(open(file_2, "rb"), delimiter=",", skiprows=1)
    trajectory = temp

    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")  # 信息接收

    writer = SummaryWriter()
    if not os.path.exists(opt.log_folder):
        os.mkdir(opt.log_folder)
    if not os.path.exists(opt.ckpt_folder):
        os.mkdir(opt.ckpt_folder)
    # 随机种子
    torch.manual_seed(opt.seed)
    np.random.seed(opt.seed)

    env = RealEnv(rtde_r=rtde_r, trajectory=trajectory, init_point=init_angle, expect_force=-20, m=400, b=7500,
                  k=500)

    state_dim = 3  # 状态空间两个值 delta X 和 delta F,还有Xe
    action_dim = 2  # 动作空间也是两个值 delta B 和 delta F
    print('Real Environment\nState Size: {}\nAction Size: {}\n'.format(state_dim, action_dim))
    if opt.mode == 'train':
        train(env, state_dim, action_dim, solved_reward=opt.solved_reward,
              max_episodes=opt.max_episodes, max_timesteps=opt.max_timesteps, update_timestep=opt.update_timesteps,
              action_std=opt.action_std, action_std_decay_rate=opt.action_std_decay_rate,
              min_action_std=opt.min_action_std, K_epochs=opt.K_epochs, eps_clip=opt.eps_clip,
              gamma=opt.gamma, lr=opt.lr, betas=[0.9, 0.990], ckpt_folder=opt.ckpt_folder, log_folder=opt.log_folder,
              restore=opt.restore, tb=opt.tb, print_interval=opt.print_interval, save_interval=opt.save_interval)
    elif opt.mode == 'test':
        test(env, state_dim, action_dim,
             action_std=opt.action_std, K_epochs=opt.K_epochs, eps_clip=opt.eps_clip,
             gamma=opt.gamma, lr=opt.lr, betas=[0.9, 0.990], ckpt_folder=opt.ckpt_folder, test_episodes=1)
    else:
        raise Exception("Wrong Mode!")
    if opt.tb:
        writer.close()

    l = np.array(force_list)
    length = [i * 0.02 for i in range(len(l))]
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.plot(length, l)

    r = np.array(reward_list)
    length = [i for i in range(len(r))]
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.plot(length, r)
    np.savetxt("force_train.csv",l,'%.6f')
    plt.show()
