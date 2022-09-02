# Franka Experiments

Deploy the method on the real Franka robot, including the vision implementation with realsense.

## Install 

Make sure you have [Franka Control Interface (FCI)](https://frankaemika.github.io/docs/) installed.

## Run
- Run Franka

- Run Moveit and follower
  ```
  roslaunch rl_interface main.launch
  ```
## Trainning

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