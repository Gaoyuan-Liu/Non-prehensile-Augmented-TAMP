# Non-Prehensile Augmented TAMP

Robotic manipulation in cluttered environments requires synergistic planning among prehensile and non-prehensile actions. Previous work on sampling-based Task and Motion Planning (TAMP) algorithms, e.g. PDDLStream, provide a fast and generalizable solution for multi-modal manipulation. However, they are likely to fail in cluttered scenarios where no collision-free grasping approaches can be sampled without preliminary manipulations.
To extend the ability of sampling-based algorithms, we integrate a vision-based Reinforcement Learning (RL) non-prehensile procedure, namely pusher, the pushing actions generated by pusher can eliminate interlocked situations and make the problem solvable. Also, the sampling-based algorithm evaluates the pushing actions by providing rewards in the training process, thus the pusher can learn to avoid situations containing irreversible failures. 
The proposed hybrid planning method is validated on a cluttered bin picking problem and implemented in both simulation and real world. Results show that the pusher can effectively improve the success ratio of the previous sampling-based algorithm, while the sampling-based algorithm can help the pusher to learn pushing skills.

<img src="https://github.com/Gaoyuan-Liu/Non-prehensile-Augmented-TAMP/blob/main/pics/unsolvable.png" width="200" />

## Video
The method introduction and experiments:

[![Watch the video](https://github.com/Gaoyuan-Liu/Non-prehensile-Augmented-TAMP/blob/main/pics/youtube_logo.png)](https://youtu.be/0NF56RZ0H0c)


## Install
- Clone this repo:
  ```
  git clone git@github.com:Gaoyuan-Liu/panda_tamp.git
  ```
- Complie DownwardFast:
  ```
  cd panda_tamp/src/pddlstream

  ./downward/build.py
  ```
- Compile IKFast:
  ```
  cd panda_tamp/src/pddlstream/examples/pybullet/utils/

  pybullet-planning$ (cd pybullet_tools/ikfast/franka_panda; python setup.py)
  ```

## Run
- Nvigate terminal to `src/panda_pddlstream`
  ```
  conda activate pddlstream
  ```

- Run panda_pddl in pybullet:
  ```
  python -m examples.pybullet.panda.run_pybullet -n 3 -v
  ```

- Run panda_pddl with Franka:
  ```
  roslaunch panda_control franka_following.launch 

  python -m examples.pybullet.panda.run -n 3 -v
  ```

## Trainning
- Run moveit motion planner, go to to `ws_moveit` workspace
  ```
  source devel/setup.bash
  
  roslaunch panda_moveit_config demo.launch
  ```
- Run trainning scripts, go to `src/pddlstream/examples/pybullet/panda`
  

## ROS Interpretation
After PDDLStream solve the problem, the `solution` after post process returns a list `commands`, the elements in the list are classes defined in `panda_primitives`. Therefore, the main pourpose of ROS interpretation is to interpret the `panda_primitives` to ROS commands. 

## Debug

### General 
1. The defaut top grasping pose is in `panda_utils.py`.

### Moveit cartesian path
The [post](https://thomasweng.com/moveit_cartesian_jump_threshold/)

### Pybullet camera
The [post](https://towardsdatascience.com/simulate-images-for-ml-in-pybullet-the-quick-easy-way-859035b2c9dd)

### Test