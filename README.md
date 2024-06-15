# Non-Prehensile Augmented TAMP

Robotic manipulation in cluttered environments requires synergistic planning among prehensile and non-prehensile actions. Previous work on sampling-based Task and Motion Planning (TAMP) algorithms, e.g. PDDLStream, provide a fast and generalizable solution for multi-modal manipulation. However, they are likely to fail in cluttered scenarios where no collision-free grasping approaches can be sampled without preliminary manipulations.
To extend the ability of sampling-based algorithms, we integrate a vision-based Reinforcement Learning (RL) non-prehensile procedure, namely pusher, the pushing actions generated by pusher can eliminate interlocked situations and make the problem solvable. Also, the sampling-based algorithm evaluates the pushing actions by providing rewards in the training process, thus the pusher can learn to avoid situations containing irreversible failures. 
The proposed hybrid planning method is validated on a cluttered bin picking problem and implemented in both simulation and real world. Results show that the pusher can effectively improve the success ratio of the previous sampling-based algorithm, while the sampling-based algorithm can help the pusher to learn pushing skills.

<img src="https://github.com/Gaoyuan-Liu/Non-prehensile-Augmented-TAMP/blob/main/pics/intro.png" width="400" />

## Video
The method introduction and experiments:

[![Watch the video](https://github.com/Gaoyuan-Liu/Non-prehensile-Augmented-TAMP/blob/main/pics/video.png)](https://youtu.be/uygLfFD1Di8)


## Install
- Clone this repo.

- Complie DownwardFast:
  ```
  cd src/pddlstream

  ./downward/build.py
  ```
- Compile IKFast:
  ```
  cd src/utils/pybullet_tools/ikfast/franka_panda
  
  python setup.py
  ```

## Run
- Nvigate terminal to `src/pusher`

- Run TAMP solver demo in pybullet:
  ```
  cd src/pusher

  python run_pybullet -n 3 -v
  ```

## Trainning
- Run moveit motion planner, go to to `ws_moveit` workspace
  ```
  source devel/setup.bash
  
  roslaunch panda_moveit_config demo.launch
  ```

- Run trainning scripts, go to `src/pusher/`
  ```
  python train.py
  ```

## Contribution

