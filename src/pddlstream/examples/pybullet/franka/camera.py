from __future__ import print_function
from turtle import pos


import numpy as np
import time
import os
import sys
from matplotlib import pyplot as plt
import math

file_path = os.path.dirname(os.path.realpath(__file__))

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
from examples.pybullet.utils.pybullet_tools.panda_problems import create_panda, create_table, Problem
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
from scipy.linalg import norm

from scenario import scenario

from numpy import zeros, newaxis

viewMatrix = p.computeViewMatrix(
                cameraEyePosition=[0.5, 0, 0.6],
                cameraTargetPosition=[0.5, 0, 0],
                cameraUpVector=[1, 0, 0])

projectionMatrix = p.computeProjectionMatrixFOV(
                fov= 65,#math.atan(0.5) * (180/math.pi) * 2 #
                aspect= 4/3,#1.0,
                nearVal= 0.1,
                farVal=1)

class camera():
    def __init__(self):

        # plt.ion()
        # self.fig = plt.figure()
        # self.ax = plt.subplot(1,1,1)
        # self.ax.set_xlabel('Time')
        # self.ax.set_ylabel('Value')
        # t = []
        y = []
        # self.ax.plot( t , y , 'ko-' , markersize = 10 ) # add an empty line to the plot
        # self.fig.show()

    def observe(self):
        # Camera parameters
        # viewMatrix = p.computeViewMatrix(
        #                 cameraEyePosition=[0.5, 0, 1],
        #                 cameraTargetPosition=[0.5, 0, 0],
        #                 cameraUpVector=[0, 1, 0])

        # projectionMatrix = p.computeProjectionMatrixFOV(
        #                 fov=math.atan(0.5) * (180/math.pi) * 2,
        #                 aspect=1.0,
        #                 nearVal=0.1,
        #                 farVal=1)

        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                                    width=640, #224 
                                                    height=480,
                                                    viewMatrix=viewMatrix,
                                                    projectionMatrix=projectionMatrix)
    
        image_hsv = cv2.cvtColor(rgbImg, cv2.COLOR_BGR2HSV)
        
        image_state = image_hsv[:, :, 1]
        mask = image_state >= 200

        masked_img = np.uint8(mask) * 255
        masked_img = cv2.resize(masked_img, (100, 100))

        masked_img = depthImg
        cv2.imshow('depthImg', depthImg)

        return masked_img[newaxis,:,:]

    def safety_observe(self):
        # Camera parameters
        # viewMatrix = p.computeViewMatrix(
        #                 cameraEyePosition=[0.5, 0, 1],
        #                 cameraTargetPosition=[0.5, 0, 0],
        #                 cameraUpVector=[0, 1, 0])

        # projectionMatrix = p.computeProjectionMatrixFOV(
        #                 fov=math.atan(0.5) * (180/math.pi) * 2,
        #                 aspect=1.0,
        #                 nearVal=0.1,
        #                 farVal=1)

        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                                    width=640, #224 
                                                    height=480,
                                                    viewMatrix=viewMatrix,
                                                    projectionMatrix=projectionMatrix)
    
        image_hsv = cv2.cvtColor(rgbImg, cv2.COLOR_BGR2HSV)
        
        image_state = image_hsv[:, :, 1]
        mask = image_state >= 200

        masked_img = np.uint8(mask) * 255

        # Dilation
        kernel = np.ones((5,5), np.uint8)
        dilated_img = cv2.dilate(masked_img, kernel, iterations=2)

        # print(mask)

        return dilated_img[newaxis,:,:]

    def world_to_pix(self, real_position):
        x_pix = math.trunc((1 - real_position[0]) *100)

        y_pix = math.trunc((real_position[1]+0.5) *100)

        return [x_pix, y_pix]

    def observe_obj(self, obj_dict):
        
        current_obj_dict = dict()
        for i in obj_dict:
            pose = get_pose(i)
            if pose[0][0] >= 0:
                current_obj_dict[i] = list(pose[0])

        return current_obj_dict

    def compare_images(self, img1, img2):
        # diff = img1 - img2
        # m_norm = sum(abs(diff)) # Manhattan norm
        # z_norm = norm(diff.ravel(), 0) # Zero norm

        if len(img1.shape) == 3:
            img1 = img1[0, :, :]
            img2 = img2[0, :, :]

        errorL2 = cv2.norm(img1, img2, cv2.NORM_L2)
        print(f'error = {errorL2}')
        if errorL2 <= 1000:
            same = True
        else:
            same = False
        return same
















if __name__ == '__main__':
    connect(use_gui=True)
    robot = scenario()
    # pushed_dict = pushes(robot, dict)
    camera = camera()

    image1 = camera.observe()

    # image2 = camera.observe()
    

    cv2.imshow('image', image1[0,:,:])
    cv2.imwrite('/home/liu/panda_tamp/src/pddlstream/examples/pybullet/panda/camera_obs.png',image1[0,:,:])
    cv2.waitKey(0)

    # print(world_to_pix([0, 0]))
    # print(pushed_dict)















    
