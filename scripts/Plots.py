#! /usr/bin/env python

import numpy as np
from math import *
import matplotlib.pyplot as plt

import sys
DATA_PATH = '/home/duynam/turtlebot_ws/src/turtlebot3_nav_q_feedback/Data'
sys.path.insert(0, DATA_PATH)

# Plot learning parameters
def plot_learning(log_file_dir):
    reward_per_episode = np.genfromtxt(log_file_dir+'/reward_per_episode.csv', delimiter = ' , ')
    steps_per_episode = np.genfromtxt(log_file_dir+'/steps_per_episode.csv', delimiter = ' , ')
    T_per_episode = np.genfromtxt(log_file_dir+'/T_per_episode.csv', delimiter = ' , ')

    reward_min_per_episode = np.genfromtxt(log_file_dir+'/reward_min_per_episode.csv', delimiter = ' , ')
    reward_max_per_episode = np.genfromtxt(log_file_dir+'/reward_max_per_episode.csv', delimiter = ' , ')
    reward_avg_per_episode = np.genfromtxt(log_file_dir+'/reward_avg_per_episode.csv', delimiter = ' , ')

    accumulated_reward = np.array([])
    av_steps_per_episodes = np.array([])
    episodes = np.arange(len(reward_per_episode))

    # Accumulated rewards and average steps
    for i in range(len(episodes)):
        accumulated_reward = np.append(accumulated_reward, np.sum(reward_per_episode[:i]))
        av_steps_per_episodes = np.append(av_steps_per_episodes, steps_per_episode[i])

    # plt.style.use('seaborn-ticks')

    plt.figure(1)
    # plt.subplot(221)
    plt.plot(reward_per_episode)
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.title('Total reward per episode')
    plt.grid()

    plt.figure(2)
    # plt.subplot(222)
    plt.plot(steps_per_episode)
    plt.xlabel('Episode')
    plt.ylabel('Steps')
    plt.title('Steps per episode')
    plt.ylim(np.min(steps_per_episode) - 10, np.max(steps_per_episode) + 10)
    plt.grid()

    plt.figure(3)
    # plt.subplot(223)
    plt.plot(T_per_episode)
    plt.xlabel('Episode')
    plt.ylabel('T')
    plt.title('T per episode')
    plt.grid()

    plt.figure(4)
    # plt.subplot(224)
    plt.plot(reward_max_per_episode, label = 'max rewards')
    plt.plot(reward_avg_per_episode, label = 'avg rewards')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.legend()
    plt.title('Reward per episode')
    plt.grid()

    plt.figure(5)
    plt.plot(episodes, accumulated_reward)
    plt.xlabel('Episode')
    plt.ylabel('Accumulated reward')
    plt.title('Accumulated reward per episodes')
    plt.ylim(np.min(accumulated_reward) - 500 , np.max(accumulated_reward) + 500)
    plt.xlim(np.min(episodes), np.max(episodes))
    plt.grid()

    plt.tight_layout()

    plt.show()

plot_learning(DATA_PATH + '/Log_learning')
