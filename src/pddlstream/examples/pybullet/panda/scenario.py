from __future__ import print_function
import imp
from turtle import color, pos


import numpy as np
import time
import os
import sys
from matplotlib import pyplot as plt
import math

file_path = os.path.dirname(os.path.realpath(__file__))
# os.chdir(file_path + '/../../../')
# print(os.getcwd())

sys.path.insert(0, file_path + '/../../../')

# from examples.pybullet.utils.pybullet_tools.pr2_problems import create_pr2, create_table, Problem
# from examples.pybullet.utils.pybullet_tools.pr2_utils import get_other_arm, get_carry_conf, set_arm_conf, open_arm, \
#     arm_conf, REST_LEFT_ARM, close_arm, set_group_conf

from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, disconnect, \
    get_joint_positions, HideOutput, LockRenderer, wait_for_user

from examples.pybullet.utils.pybullet_tools.utils import get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, joint_from_name, get_point, wait_for_user, set_euler, \
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY, joint_controller, joint_controller_hold, step_simulation, control_joints


# for panda
from examples.pybullet.utils.pybullet_tools.panda_problems import create_panda, create_safe_panda, create_table, Problem, create_bin
from examples.pybullet.utils.pybullet_tools.panda_utils import get_other_arm, get_carry_conf, set_arm_conf, open_arm, \
     arm_conf, REST_LEFT_ARM, close_arm, set_group_conf, get_gripper_joints

from examples.pybullet.utils.pybullet_tools.utils import add_data_path, connect, dump_body, disconnect, wait_for_user, joints_from_names, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, link_from_name, get_link_pose, \
    multiply, Pose, Point, Euler, interpolate_poses, HideOutput, draw_pose, set_camera_pose, load_pybullet, plan_joint_motion, \
    assign_link_colors, add_line, point_from_pose, remove_handles, BLUE, TAN, INF, BodySaver

from examples.pybullet.utils.pybullet_tools.panda_primitives import Conf, Trajectory, Commands, State, control_commands

from examples.pybullet.utils.pybullet_tools.ikfast.franka_panda.ik import panda_inverse_kinematics_background

import pybullet as p
from examples.pybullet.utils.pybullet_tools.transformations import quaternion_from_euler

import rospy
import moveit_commander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
import cv2





class Scenario:
    def __init__(self, num=6) -> None:
        print('Create a scenario.')
        self.num = num

        # ----
        # Bin
        # ----
        bin_center = [0.5, 0.0]
        bin_size = [0.26, 0.38, 0.05]
        bin_thickness = 0.05
        mass = 5
        bin_front = create_box(bin_thickness, bin_size[1]+2*bin_thickness, 0.05, mass=mass, color=BROWN)
        bin_back = create_box(bin_thickness, bin_size[1]+2*bin_thickness, 0.05, mass=mass, color=BROWN)
        set_point(bin_front, Point(x=bin_center[0]+bin_size[0]/2+bin_thickness/2, y=bin_center[1], z=0.025))
        set_point(bin_back, Point(x=bin_center[0]-bin_size[0]/2-bin_thickness/2, y=bin_center[1], z=0.025))

        bin_right = create_box(bin_size[0], bin_thickness, 0.05, mass=mass, color=BROWN)
        bin_left = create_box(bin_size[0], bin_thickness, 0.05, mass=mass, color=BROWN)
        set_point(bin_right, Point(x=bin_center[0], y=bin_center[1]+bin_size[1]/2+bin_thickness/2, z=0.025))
        set_point(bin_left, Point(x=bin_center[0], y=bin_center[1]-bin_size[1]/2-bin_thickness/2, z=0.025))

        # bin_bottom = create_box(0.27, 0.39, 0.001, color=BROWN)
        # set_point(bin_bottom, Point(x=bin_center[0], y=bin_center[1], z=0.0005))
        self.bin_poses = dict(((bin_front, [bin_center[0]+bin_size[0]/2+bin_thickness/2, bin_center[1], 0.025]),
                                (bin_back, [bin_center[0]-bin_size[0]/2-bin_thickness/2, bin_center[1], 0.025]),
                                (bin_right, [bin_center[0], bin_center[1]+bin_size[1]/2+bin_thickness/2, 0.025]),
                                (bin_left, [bin_center[0], bin_center[1]-bin_size[1]/2-bin_thickness/2, 0.025])))

        self.bin = [bin_front, bin_back, bin_right, bin_left]

    def scenario(self, num=6):
        p.setGravity(0,0,-10)
        add_data_path()
        floor = load_pybullet("plane.urdf")
        p.changeDynamics(floor, -1, lateralFriction=1)

        # ------
        # Robot
        # ------
        panda = create_panda()
        robot = panda
        
        # ----
        # Bin
        # ----
        # bin_center = [0.5, 0.0]
        # bin_size = [0.26, 0.38, 0.05]
        # bin_thickness = 0.05
        # mass = 5
        # bin_front = create_box(bin_thickness, bin_size[1]+2*bin_thickness, 0.05, mass=mass, color=BROWN)
        # bin_back = create_box(bin_thickness, bin_size[1]+2*bin_thickness, 0.05, mass=mass, color=BROWN)
        # set_point(bin_front, Point(x=bin_center[0]+bin_size[0]/2+bin_thickness/2, y=bin_center[1], z=0.025))
        # set_point(bin_back, Point(x=bin_center[0]-bin_size[0]/2-bin_thickness/2, y=bin_center[1], z=0.025))

        # bin_right = create_box(bin_size[0], bin_thickness, 0.05, mass=mass, color=BROWN)
        # bin_left = create_box(bin_size[0], bin_thickness, 0.05, mass=mass, color=BROWN)
        # set_point(bin_right, Point(x=bin_center[0], y=bin_center[1]+bin_size[1]/2+bin_thickness/2, z=0.025))
        # set_point(bin_left, Point(x=bin_center[0], y=bin_center[1]-bin_size[1]/2-bin_thickness/2, z=0.025))

        # # bin_bottom = create_box(0.27, 0.39, 0.001, color=BROWN)
        # # set_point(bin_bottom, Point(x=bin_center[0], y=bin_center[1], z=0.0005))
        # bin_poses = ((bin_front, [bin_center[0]+bin_size[0]/2+bin_thickness/2, bin_center[1], 0.025]),
        #             (bin_back, [bin_center[0]+bin_size[0]/2+bin_thickness/2, bin_center[1], 0.025]),
        #             (bin_right, [bin_center[0], bin_center[1]+bin_size[1]/2+bin_thickness/2, 0.025]),
        #             (bin_left, [bin_center[0], bin_center[1]-bin_size[1]/2-bin_thickness/2, 0.025]))

        
        # --------
        # Objects
        # --------
        obj_poses_list = []
        block_collision_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
        block_visual_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025], rgbaColor=[0,1,0,1])
        for i in range(num):
            # dic_list.append((i + 1, np.zeros((2, 3), dtype=np.float64)))

            box = p.createMultiBody(baseMass = 1,
                                    baseCollisionShapeIndex = block_collision_id,
                                    baseVisualShapeIndex = block_visual_id,
                                    basePosition = [-0.6, 0.0, 0.025])
            # Change the coefficient of friction
            p.changeDynamics(box, -1, lateralFriction=1)
            obj_poses_list.append((box, [0.6, 0.0, 0.03]))

        obj_poses = dict(obj_poses_list)
        # obj_poses = reset_obj(obj_poses)
        return robot, obj_poses

    def reset_obj_interlock(self, obj_poses):
        num = len(obj_poses)

        # The first 4
        origin = [np.random.uniform(low=0.5-0.01, high=0.5+0.01), #0.3 -0.7
                np.random.uniform(low=-0.01, high=0.01),
                0.03]

        lock_center = origin #[origin[0]+0.03, origin[1]+0.03, 0.03]
        p0 = [origin[0]-0.03, origin[1]-0.03, 0.03] #[0.6, -0.5, 0.03]
        p1 = [p0[0], p0[1]+0.06, 0.03]
        p2 = [p1[0]+0.06, p0[1], 0.03]
        p3 = [p1[0]+0.06, p0[1]+0.06, 0.03]
        

        origin_2d = origin[:2]
        angle = np.random.uniform(low=-math.pi, high=math.pi)#math.pi/4 (rand)
        angle_quat = quaternion_from_euler(0, 0, angle)
        p0 = self.rotate(origin_2d, p0[:2], angle) + [0.03]
        p1 = self.rotate(origin_2d, p1[:2], angle) + [0.03]
        p2 = self.rotate(origin_2d, p2[:2], angle) + [0.03]
        p3 = self.rotate(origin_2d, p3[:2], angle) + [0.03]
        lock_center = self.rotate(origin_2d, lock_center[:2], angle) + [0.03]

        p.resetBasePositionAndOrientation(list(obj_poses.keys())[0], p0, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[1], p1, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[2], p2, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[3], p3, angle_quat)
        

        # The rest
        random_x = np.random.uniform(low = lock_center[0]-0.05, high=lock_center[0]+0.05, size=num-4)
        random_y = np.random.uniform(low = lock_center[1]-0.05, high=lock_center[1]+0.05, size=num-4)
        
        for i in range(num-4):
            p_i = [random_x[i], random_y[i], 0.03]
            p.resetBasePositionAndOrientation(list(obj_poses.keys())[4+i], p_i, angle_quat)

        for i in range(2000):
            step_simulation()

        return obj_poses



        # for i in range(2000):
        #     step_simulation()


    def reset_obj_pop(self, obj_poses):
        
        num = len(obj_poses)

        # The first 4
        origin = [np.random.uniform(low=0.3, high=0.5),
                np.random.uniform(low=-0.2, high=0.2),
                0.03]

        # # The rest
        # random_x = np.random.uniform(low = lock_center[0]-0.05, high=lock_center[0]+0.05, size=num-4)
        # random_y = np.random.uniform(low = lock_center[1]-0.05, high=lock_center[1]+0.05, size=num-4)
        p.setRealTimeSimulation(1)

        for i in range(num):
            angle = np.random.uniform(low=-math.pi, high=math.pi)#math.pi/4 (rand)
            angle_quat = quaternion_from_euler(0, 0, angle)
            p_i = [0.5, 0.0, 0.03]
            p.resetBasePositionAndOrientation(list(obj_poses.keys())[i], p_i, angle_quat)
            time.sleep(1)





        p.setRealTimeSimulation(0)
        # for i in range(2000):
        #     step_simulation()

        return obj_poses


    def reset_obj_solvable(self, obj_poses):
        p0 = [0.4, -0.3, 0.03]
        p1 = [0.4, -0.2, 0.03]
        p2 = [0.4, -0.1, 0.03]
        p3 = [0.4, -0.0, 0.03]
        p4 = [0.4, 0.1, 0.03]
        p5 = [0.4, 0.2, 0.03]
        angle_quat = [0,0,0,1]
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[0], p0, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[1], p1, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[2], p2, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[3], p3, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[4], p4, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[5], p5, angle_quat)

    def reset_obj_unsolvable(self, obj_poses):
        num = len(obj_poses)

        # The first 4
        origin = [np.random.uniform(low=0.5-0.01, high=0.5+0.01), #0.3 -0.7
                np.random.uniform(low=-0.01, high=0.01),
                0.03]

        lock_center = origin #[origin[0]+0.03, origin[1]+0.03, 0.03]
        p0 = [origin[0]-0.025, origin[1]-0.025, 0.03] #[0.6, -0.5, 0.03]
        p1 = [p0[0], p0[1]+0.051, 0.03]
        p2 = [p1[0]+0.051, p0[1], 0.03]
        p3 = [p1[0]+0.051, p0[1]+0.051, 0.03]
        

        origin_2d = origin[:2]
        angle = np.random.uniform(low=-math.pi, high=math.pi)#math.pi/4 (rand)
        angle = 0
        angle_quat = quaternion_from_euler(0, 0, angle)
        p0 = self.rotate(origin_2d, p0[:2], angle) + [0.03]
        p1 = self.rotate(origin_2d, p1[:2], angle) + [0.03]
        p2 = self.rotate(origin_2d, p2[:2], angle) + [0.03]
        p3 = self.rotate(origin_2d, p3[:2], angle) + [0.03]
        lock_center = self.rotate(origin_2d, lock_center[:2], angle) + [0.03]

        p.resetBasePositionAndOrientation(list(obj_poses.keys())[0], p0, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[1], p1, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[2], p2, angle_quat)
        p.resetBasePositionAndOrientation(list(obj_poses.keys())[3], p3, angle_quat)
        

        # The rest
        random_x = np.random.uniform(low = lock_center[0]-0.05, high=lock_center[0]+0.05, size=num-4)
        random_y = np.random.uniform(low = lock_center[1]-0.05, high=lock_center[1]+0.05, size=num-4)
        
        for i in range(num-4):
            p_i = [random_x[i], random_y[i], 0.03]
            p.resetBasePositionAndOrientation(list(obj_poses.keys())[4+i], p_i, angle_quat)

        for i in range(2000):
            step_simulation()

        return obj_poses



        ############################3
        # p0 = [0.5, 0.165, 0.03]
        # p1 = [0.55, 0.165, 0.03]
        # p2 = [0.5, -0.115, 0.03]
        # p3 = [0.55, -0.115, 0.03]
        # p4 = [0.4, 0.1, 0.03]
        # # p5 = [0.4, 0.2, 0.03]
        # angle_quat = [0,0,0,1]
        # p.resetBasePositionAndOrientation(list(obj_poses.keys())[0], p0, angle_quat)
        # p.resetBasePositionAndOrientation(list(obj_poses.keys())[1], p1, angle_quat)
        # p.resetBasePositionAndOrientation(list(obj_poses.keys())[2], p2, angle_quat)
        # p.resetBasePositionAndOrientation(list(obj_poses.keys())[3], p3, angle_quat)
        # p.resetBasePositionAndOrientation(list(obj_poses.keys())[4], p4, angle_quat)
        # p.resetBasePositionAndOrientation(list(obj_poses.keys())[5], p5, angle_quat)


    def reset_obj_uniform(self, obj_poses):
        for i in obj_poses:
            random_x = np.random.uniform(low=0.45, high=0.55) # 8: [0.35, 0.6][-0.15, 0.15] 
            random_y = np.random.uniform(low=-0.05, high=0.05)
            angle = np.random.uniform(low=-math.pi, high=math.pi) #math.pi/4 (rand)
            angle_quat = quaternion_from_euler(0, 0, angle)
            
            p.resetBasePositionAndOrientation(i, [random_x, random_y, 0.03], angle_quat)

        p.setRealTimeSimulation(1)
        time.sleep(0.5)
        p.setRealTimeSimulation(0)
        # for i in range(2000):
        #     step_simulation()

    def reset_obj_known(self, obj_poses):
        for i in obj_poses:
            p.resetBasePositionAndOrientation(i, obj_poses[i][0], obj_poses[i][1])

    def reset_bin(self):
        for i in self.bin_poses:
            p.resetBasePositionAndOrientation(i, self.bin_poses[i], [0, 0, 0, 1])
            # set_point(i, Point(x=self.bin_poses[i][0], y=self.bin_poses[i][1], z=self.bin_poses[i][2]))


    # ---------------
    # Dirty Functions
    # ---------------
    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return [qx, qy]





if __name__ == '__main__':
    connect(use_gui=True)
    scenario = Scenario()
    robot, obj_poses = scenario.scenario(4)
    joints = get_movable_joints(robot)
    # Config is (joint1, joint2, ..., joint7, )
    initial_conf = (0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0, 0.02, 0.02)
    
    arm = 'panda'
    set_arm_conf(robot, joints, initial_conf)

    for i in range(20):
        print(scenario.bin_poses)
        scenario.reset_bin()
        scenario.reset_obj_interlock(obj_poses)
        
        
    #reset_obj_pop(obj_poses)

        wait_for_user()

    # print(quaternion_from_euler(0,0,0))










