from codecs import ignore_errors
import os
import sys
from matplotlib.pyplot import plot_date

from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.env_util import make_vec_env

# Import panda training environment
from rl_env_push import PushEnv


import rospy
import gym
from gym import wrappers
from std_msgs.msg import Float64
import numpy as np
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
import pandas as pd

class TensorboardCallback(BaseCallback):
  """
  Custom callback for plotting additional values in tensorboard.
  """

  def __init__(self, verbose=0):
    super(TensorboardCallback, self).__init__(verbose)

  def _on_step(self) -> bool:
    # Log scalar value (here a random variable)
    value = self.training_env.get_attr('total_replan_times')[0]
    # print(value)
    self.logger.record('total_replan_times', value)
    return True

if __name__ == "__main__":
  # Set the current dir
  outdir = os.path.dirname(os.path.abspath(__file__))
  os.chdir(outdir)

  rospy.init_node('cobot_training')

  env = gym.make('PandaRlEnvironmentPush-v0')
  
  #env = wrappers.Monitor(env, outdir, force=True)
  env = Monitor(env, outdir)

  # RUN_TIME = 1

  # Change 1
  model = PPO.load("ppo_panda_1", env=env, tensorboard_log="./tensorboard/")
  model.set_env(env)

  env.reset()

  # Change 2
  model.learn(total_timesteps=10000, tb_log_name="continue_run_1", reset_num_timesteps=False)
  # Change 3
  model.save("ppo_panda_2")

  # Save data
  # Change 4
  plot_date_1 = pd.read_csv('plot_data_1.csv')
  plot_date_2 = env.plot_data
  plot_data = plot_date_1.append(plot_date_2, ignore_index=True)

  # Change 5
  plot_data.to_csv('plot_data_2.csv')

  del model
  