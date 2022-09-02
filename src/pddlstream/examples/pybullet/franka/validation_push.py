import os
import sys

from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO


# Import panda training environment
from rl_env_push import PushEnv


import rospy
import gym
from gym import wrappers

import pandas as pd




# class TensorboardCallback(BaseCallback):
#   """
#   Custom callback for plotting additional values in tensorboard.
#   """

#   def __init__(self, verbose=0):
#     super(TensorboardCallback, self).__init__(verbose)

#   def _on_step(self) -> bool:
#     # Log scalar value (here a random variable)
#     value = self.training_env.get_attr('collision_times')[0]
#     self.logger.record('collision_times', value)
#     return True

if __name__ == "__main__":
  # Set the current dir
  outdir = os.path.dirname(os.path.abspath(__file__))
  os.chdir(outdir)

  rospy.init_node('cobot_training')

  env = gym.make('PandaRlEnvironmentPush-v0')
  
  env = Monitor(env, outdir)

  
  model = PPO.load("ppo_panda", env=env, tensorboard_log="./tensorboard/")
  model.set_env(env)

  obs = env.reset()
  success_times = 0

  total_total_attempt = 0
  total_total_replan_times = 0



  for i in range(1000):
    action, _state = model.predict(obs, deterministic=False)
    obs, reward, dones, info = env.step(action)
    
    if dones == True:
      success_times += 1 

   
      obs = env.reset()
    
    
    if success_times >= 1:
      break

  # plot_data.to_csv('rl-picking-data.csv')


  del model