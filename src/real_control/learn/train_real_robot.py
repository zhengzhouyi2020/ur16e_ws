#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/6/6
# @Author : Zzy

import argparse
import os.path

import numpy as np
import rospy
import torch

from torch.utils.tensorboard import SummaryWriter
from src.real_control.learn.PPO_continuous import Memory, PPO
from src.real_control.learn.real_env import RealEnv

import rtde_receive

# episode 训练的批次    max_timesteps 一个批次最大训练次数 update_timesteps丿
parser = argparse.ArgumentParser(description='PyTorch PPO持续控制')
parser.add_argument('--gpus', default=1, type=int, help='gpu的数量')
parser.add_argument('--solved_reward', type=float, default=10000, help='当奖励值大于其，则训练停止')
parser.add_argument('--print_interval', type=int, default=1, help='打印批次的结果')
parser.add_argument('--save_interval', type=int, default=1, help='保存批次的模型结果')
parser.add_argument('--max_episodes', type=int, default=100, help='最大批次，不能太大，否则训练不完')
parser.add_argument('--max_timesteps', type=int, default=800, help='最大时间步长，这里应该是轨迹点的个数')
parser.add_argument('--update_timesteps', type=int, default=64, help='更新策略的步长')
parser.add_argument('--action_std', type=float, default=0.5,
                    help='多维正态分布的action标准差 (Multivariate Normal)')
parser.add_argument('--K_epochs', type=int, default=32,
                    help='update the policy for how long time everytime')  # 存疑，计算K_epochs 步后的奖励吗
parser.add_argument('--eps_clip', type=float, default=0.2, help=' p/q 裁减率')
parser.add_argument('--gamma', type=float, default=0.99, help='discount factor')
parser.add_argument('--lr', type=float, default=0.0003)
parser.add_argument('--seed', type=int, default=123, help='可变的随机种子')
parser.add_argument('--ckpt_folder', default='./checkpoints', help='模型保存的文件夹位置')
parser.add_argument('--tb', default=True, action='store_true', help='是否使用可视化')
parser.add_argument('--log_folder', default='./logs', help='日志保存文件夹的位置')
parser.add_argument('--mode', default='train', help='选择test还是train')
parser.add_argument('--restore', default=False, action='store_true', help='是否加载已有模型继续训练')
opt = parser.parse_args()


def train(env, state_dim, action_dim, solved_reward,
          max_episodes, max_timesteps, update_timestep, action_std, K_epochs, eps_clip,
          gamma, lr, betas, ckpt_folder, restore, tb=False, print_interval=1, save_interval=1):
    ckpt = ckpt_folder + '/PPO_continuous' + '.pth'
    if restore:
        print('Load checkpoint from {}'.format(ckpt))

    memory = Memory()

    ppo = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip, restore=restore, ckpt=ckpt)

    running_reward, avg_length, time_step = 0, 0, 0

    # training loop
    for i_episode in range(1, max_episodes + 1):
        state = env.reset()
        t = 0
        for t in range(max_timesteps):
            time_step += 1
            action = ppo.select_action(state, memory)
            state, reward, done, info = env.step(action)
            memory.rewards.append(reward)
            memory.is_terminals.append(done)

            if time_step % update_timestep == 0:
                ppo.update(memory)
                memory.clear_memory()
                time_step = 0

            running_reward += reward
            if done:
                break
        avg_length += t
        # 保存模型文件
        if running_reward > (print_interval * solved_reward) or i_episode % save_interval == 0:
            torch.save(ppo.policy.state_dict(), ckpt_folder + '/PPO_continuous.pth')
            print('Save a checkpoint!')
            break
        # 计算一个打印时间的各奖励和
        if i_episode % print_interval == 0:
            avg_length = int(avg_length / print_interval)
            running_reward = int((running_reward / print_interval))
            print('Episode {} \t Avg length: {} \t Avg reward: {}'.format(i_episode, avg_length, running_reward))

            if tb:
                writer.add_scalar('scalar/reward', running_reward, i_episode)
                writer.add_scalar('scalar/length', avg_length, i_episode)
            running_reward, avg_length = 0, 0


def tes(env, state_dim, action_dim, action_std, K_epochs, eps_clip, gamma, lr, betas, ckpt_folder,
        test_episodes):
    # 直接加载训练好的模型
    ckpt = ckpt_folder + '/PPO_continuous' + '.pth'
    print('Load checkpoint from {}'.format(ckpt))
    memory = Memory()
    ppo = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip, restore=True, ckpt=ckpt)

    episode_reward, time_step = 0, 0
    avg_episode_reward, avg_length = 0, 0

    # test
    for i_episode in range(1, test_episodes + 1):
        state = env.reset()
        while True:
            time_step += 1
            action = ppo.select_action(state, memory)
            state, reward, done, info = env.step(action)
            episode_reward += reward

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
    trajectory = np.loadtxt("../data/20220415_151254.csv", delimiter=',', skiprows=1)
    initial_point = np.hstack([trajectory[0, 7:13], np.zeros(6)])  # 设置初始的运动点, 关节角保证初始位置的准确性
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.11")  # 信息接收

    writer = SummaryWriter()
    if not os.path.exists(opt.ckpt_folder):
        os.mkdir(opt.ckpt_folder)
    # 随机种子
    torch.manual_seed(opt.seed)
    np.random.seed(opt.seed)

    env = RealEnv(rtde_r=rtde_r, trajectory=trajectory, init_point=initial_point, expect_force=-15, m=400, b=8000,
                  k=800)

    state_dim = 2  # 状态空间两个值 delta X 和 delta F，应该不只值
    action_dim = 2  # 动作空间也是两个值 delta B 和 delta F
    print('Real Environment\nState Size: {}\nAction Size: {}\n'.format(state_dim, action_dim))
    if opt.mode == 'train':
        train(env, state_dim, action_dim, solved_reward=opt.solved_reward,
              max_episodes=opt.max_episodes, max_timesteps=opt.max_timesteps, update_timestep=opt.update_timesteps,
              action_std=opt.action_std, K_epochs=opt.K_epochs, eps_clip=opt.eps_clip,
              gamma=opt.gamma, lr=opt.lr, betas=[0.9, 0.990], ckpt_folder=opt.ckpt_folder,
              restore=opt.restore, tb=opt.tb, print_interval=opt.print_interval, save_interval=opt.save_interval)
    elif opt.mode == 'test':
        tes(env, state_dim, action_dim,
            action_std=opt.action_std, K_epochs=opt.K_epochs, eps_clip=opt.eps_clip,
            gamma=opt.gamma, lr=opt.lr, betas=[0.9, 0.990], ckpt_folder=opt.ckpt_folder, test_episodes=1)
    else:
        raise Exception("Wrong Mode!")
    if opt.tb:
        writer.close()

