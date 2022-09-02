#! /venv3.8/bin/python


import os
from tkinter.tix import X_REGION

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

from pushing import Pusher
from scenario import Scenario
import pybullet as p
import moveit_commander

# Data Collection
import pandas as pd
from examples.pybullet.utils.pybullet_tools.utils import connect, get_client
from camera import Camera #observe, world_to_pix, observe_obj
from matplotlib import pyplot as plt

from solver import pddlstream_solver
import cv2 as cv
from moveit_scene import Moveit_Scene


max_episode_steps=100

reg = register(
    id='PandaRlEnvironmentPush-v0',
    entry_point='rl_env_push:PushEnv',
    max_episode_steps=100,
    )

class PushEnv(gym.Env):
    def __init__(self):
      super(PushEnv, self).__init__()

      # Define the mode
      self.mode = 'train' # mode: train/validation





      
      # Data collecting
      self.collision_times = 0
      self.episode_length = 0
      self.episode_return = 0
      self.invalid_action = 0
      self.solving_ratio = 0
      self.plot_data = pd.DataFrame({'el':[], 'er':[], 'ia':[], 'sr':[]}) # el: episode length; er: episode reward; ia: ivalid action; sr: solve ratio



      # Action space
      # self.action_space = spaces.Box(low=np.array([-5]*4, dtype=np.int32), high=np.array([4]*4, dtype=np.int32))
      self.action_space = spaces.MultiDiscrete([21, 31, 21, 31])

      # Obs space
      N_CHANNELS = 1 
      HEIGHT = 100
      WIDTH = 100
      self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(N_CHANNELS, HEIGHT, WIDTH), dtype=np.uint8)

      # Create Scenario
      # p.connect(p.GUI)
      connect(use_gui = True)
      self.scenario = Scenario()
      self.total_num = 5
      self.pick_num = 5
      self.robot, self.obj_poses = self.scenario.scenario(self.total_num)
      self.bin = self.scenario.bin

      # self.current_obj_dict = self.obj_dict

      self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
      self.move_group.set_planner_id('RRTstarkConfigDefault')

      self.solver = pddlstream_solver(self.robot, self.bin)

      self.camera = Camera()
      self.pusher = Pusher()
      self.scene = Moveit_Scene()



    ###########################
    #           Reset         #
    ###########################

    def reset(self):
      print("==== New episode ====")
      
      # --------
      # Observe
      # --------
      observation, _ = self.take_observation()

      # --------------
      # Reset Objects
      # --------------
      if self.mode == 'train':
        self.scenario.reset_obj_interlock(self.obj_poses)
        # self.scenario.reset_obj_unsolvable(self.obj_poses)
        
      elif self.mode == 'validation':
        self.scenario.reset_obj_uniform(self.obj_poses)
      else:
        print(f'\n{self.mode} is not a valid mode.\n')
      self.scenario.reset_bin()
      self.previous_bin_img = self.camera.observe_bin()

      # ----------------
      # Data Collection
      # ----------------
      if self.solving_ratio < 0:
        self.solving_ratio = 0
      df = pd.DataFrame({'el':[self.episode_length], 'er':[self.episode_return], 'ia':[self.invalid_action], 'sr':[self.solving_ratio]})
      self.plot_data = self.plot_data.append(df, ignore_index=True)
      self.episode_length = 0
      self.episode_return = 0
      self.invalid_action = 0
      self.solving_ratio = 0

      
      return observation

    ###########################
    #           Step          #
    ###########################

    def step(self, action):
      print(f'---- Step {self.episode_length} ----')

      # ---------------
      # Initialization
      # ---------------
      if self.episode_length == 0:
        self.previous_obj_img, self.previous_obj_poses = self.take_observation()
        # self.previous_bin_img = self.camera.observe_bin()
        

      # -----------------
      # Implement Action
      # -----------------
      # self.previous_obj_poses = obj_poses
      self.previous_obj_img, self.previous_obj_poses = self.take_observation()
      self.execute(action)
      
      # --------
      # Observe
      # --------
      state, obj_poses = self.take_observation()
      bin_img = self.camera.observe_bin()

      # -----------
      # Get Reward
      # -----------
      reward, achieve = self.get_reward(state, obj_poses, bin_img)

      # self.previous_obj_img = state
     
      # -----
      # Done
      # -----
      done = False
      self.episode_length += 1
      if achieve or self.episode_length > 10:
        done = True
      self.episode_return += reward

      return state, reward, done, {}



    ###########################
    #       Observation       #
    ###########################

    def take_observation(self):
      
      image_data = self.camera.observe()

      # Modify the current_obj_dict
      obj_poses = self.camera.observe_obj(self.obj_poses)

      return image_data, obj_poses

   


    


    # def execute(self, action):
    #   # Here the action is a int np.array, 
    #   # the first 2 numbers present pushing start point,
    #   # while the last 2 numbers presents pushing end point
    #   # pushing region is x = [0.3, 0.7] y = [-0.3, 0.3]


    #   # if np.linalg.norm([action[2]-action[0], action[3]-action[1]])>0.01:
      
    #   interval = 0.02
    #   x_1 = 0.4+ interval * action[0]
    #   y_1 = -0.15 + interval * action[1]
    #     # x_2 = 0.3 + interval * action[2]
    #     # y_2 = -0.3 + interval * action[3]

        
    #   _, obj_dict = self.take_observation()

    #   smallest_dis = 10
    #   for i in obj_dict:
    #     dis = self.distance([x_1, y_1], obj_dict[i][:2])
    #     if dis <= smallest_dis:
    #       target = i
    #       smallest_dis = dis

    #   x_2 = obj_dict[target][0]
    #   y_2 = obj_dict[target][1]

    #   x_e = x_2 + (x_2-x_1)/smallest_dis * interval
    #   y_e = y_2 + (y_2-y_1)/smallest_dis * interval

    #   x_s = x_2 - (x_2-x_1)/smallest_dis * interval * 2
    #   y_s = y_2 - (y_2-y_1)/smallest_dis * interval * 2

    #   self.pusher.push(self.robot, obj_dict, [x_s, y_s], [x_e, y_e])

    ###########################
    #        Execution        #
    ###########################
    def execute(self, action):
      # Here the action is a int np.array, 
      # the first 2 numbers present pushing start point,
      # while the last 2 numbers presents pushing end point
      # pushing region is x = [0.3, 0.7] y = [-0.3, 0.3]


      if np.linalg.norm([action[2]-action[0], action[3]-action[1]])>0.01:
      
        interval = 0.01
        x_1 = 0.40 + interval * action[0]
        y_1 = -0.15 + interval * action[1]
        x_2 = 0.40 + interval * action[2]
        y_2 = -0.15 + interval * action[3]

        _, obj_poses = self.take_observation()
        # print(f'\nThe obj_poses in env is\n{obj_poses} \n')
        self.pusher.push(self.robot, obj_poses, [x_1, y_1], [x_2, y_2])

      else:
        print('The start point and end point is overlapped.')

     

          




    ###########################
    #     Reward Functions    #
    ###########################

    def get_reward(self, state, obj_poses, bin_img): # observe_data is image, but here we need the absolut position of the blocks to solve
      # --------------------
      # Objects difference?
      # --------------------
      same = self.camera.compare_images(state, self.previous_obj_img)

      if same == True:
        print('Did not make any changes.')
        self.invalid_action += 1
        return -2, False # If the image didn't change then the solver will not try to solve 
        
      # -----------------
      # Bin cush?
      # -----------------
      bin_crashed = not self.camera.compare_images(bin_img, self.previous_bin_img, threshold=2000)
      if bin_crashed == True:
        print('Bin crashed.')
        # Put bin back
        self.scenario.reset_bin()
        # Put objects back
        self.scenario.reset_obj_interlock(self.previous_obj_poses)
        self.invalid_action += 1
        self.solving_ratio = -1
        if self.mode == 'train':
          return -10, False
        if self.mode == 'validation':
          return -10, True
      
      

      # ----------
      # Solvable?
      # ----------
      # solvable = self.solver.solve_and_execute(obj_poses, n_pick=self.pick_num)

      solution = self.solver.problem_solve(obj_poses, self.pick_num)
    

      plan, _, _ = solution
      
      if plan!=None:
          solvable = True
          print('Solvable now!')
          
      else:
          print('No solution found.')
          solvable = False

  
      if solvable == False:
        return 0, False

      else:
        if self.solving_ratio <= 0:
          self.solving_ratio = 0
        else:
          self.solving_ratio = 1
        return 10, True


        # return reward, achieve

    ###########################
    #     Dirty Functions     #
    ###########################

    def distance(self, point_1, point_2): 
      err = np.subtract(point_1, point_2)
      dist = np.linalg.norm(err)
      return dist
