# Franka Panda PDDLStream

The project is built on top of [PDDLStream](https://github.com/caelan/pddlstream).

The original version only consider pr2 robot, here we extend the experiment to a Franka Panda robot.


## Publications

* [PDDLStream: Integrating Symbolic Planners and Blackbox Samplers via Optimistic Adaptive Planning](https://arxiv.org/abs/1802.08705
)
<!--* [STRIPStream: Planning In Infinite Domains](https://arxiv.org/abs/1701.00287)-->

## Run the code
- Try panda in pybullet
```
python -m examples.pybullet.panda.run -n 5 -v
```

## ROS Interpretation
After PDDLStream solve the problem, the `solution` after post process returns a list `commands`, the elements in the list are classes defined in `panda_primitives`. Therefore, the main pourpose of ROS interpretation is to interpret the `panda_primitives` to ROS commands. 

## Debug
1. The defaut top grasping pose is in `panda_utils.py`.
