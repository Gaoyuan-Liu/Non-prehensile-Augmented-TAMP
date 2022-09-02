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
from solver import pddlstream_solver




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

  # ------------
  # Data saving
  # ------------
  plot_data = pd.DataFrame({'total_episodes':[], 'solve_time_without_pusher':[], 'solve_time_with_pusher':[]})
  solve_time_without_pusher = 0
  solve_time_with_pusher = 0
  
  # --------
  # Set Env
  # --------
  rospy.init_node('cobot_training')
  env = gym.make('PandaRlEnvironmentPush-v0')
  env = Monitor(env, outdir)
  model = PPO.load("ppo_panda", env=env, tensorboard_log="./tensorboard/")
  model.set_env(env)
  obs = env.reset()

  total_episodes = 100
  success_times = 0
  
  solver = pddlstream_solver(env.robot, env.bin)
  for i in range(total_episodes):
    print(f'\nThe {i+1} Episode.\n')

    env.reset()

    # ------------------------
    # Start by direct solving
    # ------------------------
    # solver = pddlstream_solver(env.robot)
    _, obj_poses = env.take_observation()
    n_picking = 5
    # solution = solver.problem_solve(obj_poses, n_picking)

    # plan, _, _ = solution
    plan = None

    if plan!=None:
        solvable = True
        solve_time_with_pusher += 1
        solve_time_without_pusher += 1
        print('Solvable without pusher!')
        
    else:
        print('No solution found without pusher.')
        solvable = False
    
    # -------------------------------------
    # If not solvable, use pushing augment
    # -------------------------------------
    if solvable == False:
      for i in range(10):
        action, _state = model.predict(obs, deterministic=False)
        obs, reward, dones, info = env.step(action)
        if dones == True:
          if i < 9 and reward > 0: #?
            solve_time_with_pusher += 1
            
          break


  df = pd.DataFrame({'total_episodes':[total_episodes], 'solve_time_without_pusher':[solve_time_without_pusher], 'solve_time_with_pusher':[solve_time_with_pusher]})
  plot_data = plot_data.append(df, ignore_index=True)

  plot_data.to_csv('compare_data.csv')




  # for i in range(1000):
  #   action, _state = model.predict(obs, deterministic=False)
  #   obs, reward, dones, info = env.step(action)
    
  #   if dones == True:

  #   # Data saving
  #   #   success_times += 1 

  #   #   total_attempt = env.episode_length
  #   #   total_replan_times = env.total_replan_times

  #   #   total_total_attempt += total_attempt
  #   #   total_total_replan_times += total_replan_times

  #   #   average_attempt = total_total_attempt/(success_times)
  #   #   average_replan_times = total_total_replan_times/(success_times)

  #   #   df = pd.DataFrame({'total_attempt':[total_attempt], 'total_replan_times':[total_replan_times], 'average_attempt':[average_attempt], 'average_replan_times':[average_replan_times]})
  #   #   plot_data = plot_data.append(df, ignore_index=True)



  #     obs = env.reset()
    
    
  #   if success_times > 1:
  #     break

  # plot_data.to_csv('rl-picking-data.csv')


  del model