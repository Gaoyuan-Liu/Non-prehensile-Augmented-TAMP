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
from scenario import scenario, random_distribute
import pybullet as p
import moveit_commander

# Data Collection
import pandas as pd
from examples.pybullet.utils.pybullet_tools.utils import connect, get_client
from camera import camera #observe, world_to_pix, observe_obj
from matplotlib import pyplot as plt

from solver import pddlstream_solver
import cv2 as cv


import pandas as pd



if __name__ == '__main__':
    connect(use_gui=True)

    robot, obj_poses = scenario(8)

    solver = pddlstream_solver(robot)
    camera = camera()

    pick_num = 8

    save_obj_poses = obj_poses
    for i in obj_poses:
      save_obj_poses[i] = []

    for i in range(1000):
      print(f'Now try {i} times, and have {len(save_obj_poses[2])} samples.')
      random_distribute(obj_poses)

      obj_dic = camera.observe_obj(obj_poses)

      t_start = time.time()
      solution = solver.problem_solve(obj_dic, pick_num)
      plan, _, _ = solution

      t_end = time.time()

      d_t = t_end - t_start

      print(f'This planning uses {d_t}s to solve. ')
        
      if plan!=None:
          solvable = True
          print('It is a solvable situation.')
          # self.execution(self.problem, solution)
          
      else:
          print('It is an unsolvable situation.')
          solvable = False
          for obj in obj_dic:
            save_obj_poses[obj].append(obj_dic[obj])

    
    df = pd.DataFrame.from_dict(save_obj_poses) 

    file_path = os.path.dirname(os.path.realpath(__file__))
    df.to_csv(file_path + '/distribution_data.csv', index=False)


          

          







  


        

