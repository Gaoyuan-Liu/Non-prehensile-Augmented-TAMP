from functools import lru_cache
import stable_baselines3 as sb

from stable_baselines3.ppo import MlpPolicy, CnnPolicy #, ActorCriticCnnPolicy
from gym import utils, spaces
import numpy as np

# Action space
action_space = spaces.MultiDiscrete([9, 17, 9, 17])

# Obs space
N_CHANNELS = 1 
HEIGHT = 100
WIDTH = 100
observation_space = spaces.Box(low=0, high=255,
                                    shape=(4, HEIGHT, WIDTH), dtype=np.uint8)

def lr_schedule(dummy):
    return 0.0001
cnn = CnnPolicy(observation_space=observation_space, action_space=action_space, lr_schedule=lr_schedule)

print(type(cnn))
net_arch = cnn.net_arch
fe = cnn.features_extractor_class
print(net_arch)
print(fe)