from __future__ import print_function
from turtle import pos


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
from examples.pybullet.utils.pybullet_tools.panda_problems import create_panda, create_table, Problem, create_safe_panda
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




def scenario():

    
    p.setGravity(0,0,-10)
    add_data_path()
    floor = load_pybullet("plane.urdf")
    p.changeDynamics(floor, -1, lateralFriction=1)
    # print(f'floor = {floor}')
    
    # Robot
    panda = create_panda()
    robot = panda

    # Wall
    wall = create_box(0.001, 2, 2, color=WHITE)
    set_point(wall, Point(x=-0.3, y=0, z=1))

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
    bin_poses = dict(((bin_front, [bin_center[0]+bin_size[0]/2+bin_thickness/2, bin_center[1], 0.025]),
                            (bin_back, [bin_center[0]-bin_size[0]/2-bin_thickness/2, bin_center[1], 0.025]),
                            (bin_right, [bin_center[0], bin_center[1]+bin_size[1]/2+bin_thickness/2, 0.025]),
                            (bin_left, [bin_center[0], bin_center[1]-bin_size[1]/2-bin_thickness/2, 0.025])))

    bin = [bin_front, bin_back, bin_right, bin_left]
    
    # Objects
    # num = 6
    # dic_list = []
    # block_collision_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
    # block_visual_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025], rgbaColor=[0,1,0,1])
    # for i in range(num):
    #     # dic_list.append((i + 1, np.zeros((2, 3), dtype=np.float64)))

    #     box = p.createMultiBody(baseMass = 0.5,
    #                             baseCollisionShapeIndex = block_collision_id,
    #                             baseVisualShapeIndex = block_visual_id,
    #                             basePosition = [0.6, 0.0, 0.03])
    #     # Change the coefficient of friction
    #     p.changeDynamics(box, -1, lateralFriction=1)
    #     dic_list.append((box, [0.6, 0.0, 0.03]))

    # objects_dict = dict(dic_list)

    # objects_dict = reset_obj(objects_dict)



    for i in range(1000):
        step_simulation()
    return robot

def reset_obj(objects_dict):
    
    num = len(objects_dict)

    # The first 4
    origin = [np.random.uniform(low=0.45, high=0.55),
              np.random.uniform(low=-0.1, high=0.1),
              0.03]

    p0 = origin #[0.6, -0.5, 0.03]
    p1 = [p0[0], p0[1]+0.06, 0.03]
    p2 = [p1[0]+0.06, p0[1], 0.03]
    p3 = [p1[0]+0.06, p0[1]+0.06, 0.03]
    lock_center = [p0[0]+0.03, p0[1]+0.03, 0.03]

    origin_2d = origin[:2]
    angle = np.random.uniform(low=0, high=math.pi)#math.pi/4 (rand)
    angle_quat = quaternion_from_euler(0, 0, angle)
    p0 = rotate(origin_2d, p0[:2], angle) + [0.03]
    p1 = rotate(origin_2d, p1[:2], angle) + [0.03]
    p2 = rotate(origin_2d, p2[:2], angle) + [0.03]
    p3 = rotate(origin_2d, p3[:2], angle) + [0.03]
    lock_center = rotate(origin_2d, lock_center[:2], angle) + [0.03]

    p.resetBasePositionAndOrientation(list(objects_dict.keys())[0], p0, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[1], p1, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[2], p2, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[3], p3, angle_quat)
    

    # objects_dict[list(objects_dict.keys())[0]] = [0.6, 0.0, 0.03]
    # objects_dict[list(objects_dict.keys())[1]] = [0.6, -0.06, 0.03]
    # objects_dict[list(objects_dict.keys())[2]] = [0.6-0.06, 0.0, 0.03]
    # objects_dict[list(objects_dict.keys())[3]] = [0.6-0.06, -0.06, 0.03]

    
    # The rest
    random_x = np.random.uniform(low = lock_center[0]-0.05, high=lock_center[0]+0.05, size=num-4)
    random_y = np.random.uniform(low = lock_center[1]-0.05, high=lock_center[1]+0.05, size=num-4)
    
    for i in range(num-4):
        p_i = [random_x[i], random_y[i], 0.03]
        p.resetBasePositionAndOrientation(list(objects_dict.keys())[4+i], p_i, angle_quat)

    # p4 = [random_x[0], random_y[0], 0.03]
    # p5 = [random_x[1], random_y[1], 0.03]


    # p.resetBasePositionAndOrientation(list(objects_dict.keys())[0], p0, angle_quat)
    # p.resetBasePositionAndOrientation(list(objects_dict.keys())[1], p1, angle_quat)
    # p.resetBasePositionAndOrientation(list(objects_dict.keys())[2], p2, angle_quat)
    # p.resetBasePositionAndOrientation(list(objects_dict.keys())[3], p3, angle_quat)
    # p.resetBasePositionAndOrientation(list(objects_dict.keys())[4], p4, angle_quat)
    # p.resetBasePositionAndOrientation(list(objects_dict.keys())[5], p5, angle_quat)
    
    for i in range(2000):
        step_simulation()

    return objects_dict



def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return [qx, qy]

def reset_obj_solvable(objects_dict):
    p0 = [0.4, -0.3, 0.03]
    p1 = [0.4, -0.2, 0.03]
    p2 = [0.4, -0.1, 0.03]
    p3 = [0.4, -0.0, 0.03]
    p4 = [0.4, 0.1, 0.03]
    p5 = [0.4, 0.2, 0.03]
    angle_quat = [0,0,0,1]
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[0], p0, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[1], p1, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[2], p2, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[3], p3, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[4], p4, angle_quat)
    p.resetBasePositionAndOrientation(list(objects_dict.keys())[5], p5, angle_quat)









if __name__ == '__main__':
    connect(use_gui=True)

    robot, obj_poses = scenario(8)

    for i in range(10):
        reset_obj(obj_poses)

        wait_for_user()










