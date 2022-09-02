# Non-Prehensile Augmented TAMP

The project is built on top of [PDDLStream](https://github.com/caelan/pddlstream).

We add a Reinforcement Learning (RL) non-prehensile procedure to help previous PDDLStream create solvable situations.

<img src="https://github.com/Gaoyuan-Liu/panda_tamp/blob/main/pics/unsolvable.png" width="200" />


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