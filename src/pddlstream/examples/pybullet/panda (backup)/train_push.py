import os
import sys

from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.callbacks import BaseCallback

# Import panda training environment
from rl_env_push import PushEnv


import rospy
import gym
from gym import wrappers



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

  # policy_kwargs = dict(activation_fn=th.nn.ReLU, net_arch=[dict(pi=[128, 128], vf=[128, 128])])
  model = PPO(MlpPolicy, env, learning_rate=0.0005, n_steps=100, batch_size=100, 
              verbose=1, tensorboard_log="./tensorboard/") # ent_coef=0.0001, policy_kwargs=policy_kwargs n_step = 1000, batch_size=

  checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./logs/',
                                           name_prefix='rl_model')

  model.learn(total_timesteps=50, tb_log_name="first_run")

  # model.learn(total_timesteps=5000, tb_log_name="second_run", reset_num_timesteps=False, callback=TensorboardCallback())
  model.save("ppo_panda")

  # collision_times = np.array(env.collision_times_list)

  env.plot_data.to_csv('plot_data.csv')


  del model
  