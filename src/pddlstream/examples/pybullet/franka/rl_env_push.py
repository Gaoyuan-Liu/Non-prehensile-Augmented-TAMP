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

from pushing import Pusher
from scenario import scenario, reset_obj
import pybullet as p
import moveit_commander

# Data Collection
import pandas as pd
from examples.pybullet.utils.pybullet_tools.utils import connect, get_client
from realsense import Camera
from matplotlib import pyplot as plt

from solver import pddlstream_solver
from moveit_scene import Moveit_Scene
import cv2 as cv
from ros_commander import ROS_Commander


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
      self.action_space = spaces.MultiDiscrete([21, 31, 21, 31])

      # Obs space
      N_CHANNELS = 1 
      HEIGHT = 100
      WIDTH = 100
      self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(1, HEIGHT, WIDTH), dtype=np.uint8)

      # Create Scenario
      # p.connect(p.GUI)
      connect(use_gui = True)
      self.total_num = 6
      self.pick_num = None
      self.robot = scenario()


      self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
      self.move_group.set_planner_id('RRTstarkConfigDefault')

      self.solver = pddlstream_solver(self.robot)

      # Camera
      self.camera = Camera()

      # Moveit Scene
      self.scene = Moveit_Scene()

      # Ros Commander
      self.ros_commander = ROS_Commander()

      # Pusher
      self.pusher = Pusher()


    ###########################
    #           Reset         #
    ###########################

    def reset(self):
      print("==== New episode ====")
      

      # Get observation
      observation, self.obj_poses = self.take_observation()


      

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
      if self.episode_length == 0:
        self.previous_img, self.obj_poses = self.take_observation()


      # Implement Action
      # self.scene.add_objects_collision(self.current_obj)
      self.execute(action)
      
      # State
      state, self.obj_poses = self.take_observation()
      # self.scene.remove_objects_collision(self.current_obj)

      # Reward
      reward, achieve = self.get_reward(state, self.obj_poses)
      self.previous_img = state
     
      # Done
      done = False
      self.episode_length += 1
      if achieve or self.episode_length > 10:
        done = True
    

      self.episode_return += reward

      return state, reward, done, {}



    ###########################
    #    General Functions    #
    ###########################

    def take_observation(self):

      self.ros_commander.go_to_observe_pose()
      image_data = self.camera.observe_in_bin()

      # Modify the current_obj_dict
      obj_poses = self.camera.get_positions()

      return image_data, obj_poses

   


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
      
        # x_1 = 0.3 + (0.6 - 0.3)/10 * action[0]
        # y_1 = 0.1 * (action[1] - 5)
        # x_2 = 0.3 + (0.6 - 0.3)/10 * action[2]
        # y_2 = 0.1 * (action[3] - 5)

        interval = 0.01
        x_1 = 0.40 + interval * action[0]
        y_1 = -0.15 + interval * action[1]
        x_2 = 0.40 + interval * action[2]
        y_2 = -0.15 + interval * action[3]

        _, obj_poses = self.take_observation()
        # print(f'\nThe obj_poses in env is\n{obj_poses} \n')
        self.pusher.push(self.robot, obj_poses, [x_1, y_1], [x_2, y_2])


       
        
          




    ###########################
    #     Reward Functions    #
    ###########################

    def get_reward(self, state, current_obj_dict): # observe_data is image, but here we need the absolut position of the blocks to solve
      
      # Image difference?
      # cv.imshow('state', state)
      # cv.imshow('previous_img', self.previous_img)
      # cv.waitKey(0)
      # same = self.camera.compare_images(state, self.previous_img)
      same = False
      if same == True:
        print('Did not make any changes.')
        return -2, False # If the image didn't change then the solver will not try to solve 
        

      # Solvable?
      else:
        solved = self.solver.solve_and_execute(current_obj_dict, self.pick_num)
          
        if solved == False:
          return 0, False

        else:
          print('Solvable now!')
          return 10, True


        # return reward, achieve
