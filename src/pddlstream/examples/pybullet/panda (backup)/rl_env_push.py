#! /venv3.8/bin/python

import os

import gym
from gym import utils, spaces
from gym.utils import seeding
from gym.envs.registration import register
from gym import spaces
from numpy.core.fromnumeric import size
import rospy
import rospkg
import time
import numpy as np
import time

import sys

from pushing import push, camera, scenario
import pybullet as p
import moveit_commander

# Data Collection
import pandas as pd
from examples.pybullet.utils.pybullet_tools.utils import connect
from camera import observe, world_to_pix
from matplotlib import pyplot as plt


max_episode_steps=100

reg = register(
    id='PandaRlEnvironmentPush-v0',
    entry_point='rl_env_push:PushEnv',
    max_episode_steps=100,
    )

class PushEnv(gym.Env):
    def __init__(self):
      super(PushEnv, self).__init__()

      self.time_fraction = 0.05

      # Reward function parameters
      self.distance_rew_factor = 100
      self.achieve_reward = 100
      self.collision_punishment = 0

      
      # Data collecting
      self.collision_times = 0
      self.episode_length = 0
      self.episode_return = 0
      self.plot_data = pd.DataFrame({'ct':[], 'el':[], 'er':[], 't':[]})
      self.t_start = time.time()
      self.t_now = time.time()


      # Action space
      # self.action_space = spaces.Box(low=np.array([-5]*4, dtype=np.int32), high=np.array([4]*4, dtype=np.int32))
      self.action_space = spaces.MultiDiscrete([11, 10, 11, 10])

      # Obs space
      N_CHANNELS = 1 
      HEIGHT = 100
      WIDTH = 100
      self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(HEIGHT, WIDTH), dtype=np.uint8)

      # Create Scenario
      # p.connect(p.GUI)
      connect(use_gui = True)
      self.robot, self.blocks = scenario()

      self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
      self.move_group.set_planner_id('RRTstarkConfigDefault')







    ###########################
    #           Reset         #
    ###########################

    def reset(self):
      print("==== New episode ====")
      

      # Get observation
      observation = self.take_observation()
     
      # Data collection
      self.t_now = time.time()
      df = pd.DataFrame({'ct':[self.collision_times], 'el':[self.episode_length], 'er':[self.episode_return], 't':[self.t_now-self.t_start]})
      self.plot_data = self.plot_data.append(df, ignore_index=True)
      self.episode_length = 0
      self.episode_return = 0
      self.collision_times = 0
      self.step_count = 0
      

    

      return observation

    ###########################
    #           Step          #
    ###########################

    def step(self, action):
      print(f'---- Step {self.episode_length} ----')

      # Implement Action
      print(action)
      print(type(action))

      self.execute(action)
      
      # State
      state = self.take_observation()

      # Reward
      reward, achieve = self.get_reward(state)
     
      # Done
      done = False
      self.episode_length += 1
      if achieve or self.episode_length > 100:
        done = True
      # if abs(state[3]) > 0.6 or abs(state[4]) > 0.5: 
      #   done = True

      self.episode_return += reward

      return state, reward, done, {}



    ###########################
    #    General Functions    #
    ###########################

    def take_observation(self):
      
      observed_data = observe()
      return observed_data

   


    def distance(self, point_1, point_2): 
      err = np.subtract(point_1, point_2)
      dist = np.linalg.norm(err)
      return dist

    


    def execute(self, action):
      # Here the action is a int np.array, 
      # the first 2 numbers present pushing start point,
      # while the last 2 numbers presents pushing end point
      # pushing region is x = [-0.5, 0.5] y = [0.3, 0.6]

      

      
      # if action[0] != [2] or action[1] != [3]:
      if np.linalg.norm([action[2]-action[0], action[3]-action[1]])>0.01:
        print(f'action = {action}')
      
        x_1 = 0.3 + (0.6 - 0.3)/10 * action[0]
        y_1 = 0.1 * (action[1] - 5)
        x_2 = 0.3 + (0.6 - 0.3)/10 * action[2]
        y_2 = 0.1 * (action[3] - 5)

        # Safety mask
        map = self.take_observation()
        
        pix_coord = world_to_pix([x_1, y_1])

        if map[pix_coord[0], pix_coord[1]] < 200: # No block is there
          
          push(self.robot, self.move_group, [x_1, y_1], [x_2, y_2])
        else:
          print(f'The invalid pushing starts from {pix_coord}.')
          # print(map[pix_coord[0], pix_coord[1]])
          # plt.imshow(map)
          # plt.show()




    ###########################
    #     Reward Functions    #
    ###########################

    def get_reward(self, observed_data):
      reward = 0
      achieve = False

      return reward, achieve
