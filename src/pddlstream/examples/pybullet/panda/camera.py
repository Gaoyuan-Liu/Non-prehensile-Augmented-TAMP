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
import cv2 as cv
from scipy.linalg import norm

from scenario import Scenario

from numpy import zeros, newaxis

# viewMatrix = p.computeViewMatrix(
#                 cameraEyePosition=[0.4, 0, 0.62],
#                 cameraTargetPosition=[0.4, 0, 0],
#                 cameraUpVector=[-1, 0, 0])

# projectionMatrix = p.computeProjectionMatrixFOV(
#                 fov= 65,#math.atan(0.5) * (180/math.pi) * 2 #
#                 aspect= 4/3,#1.0,
#                 nearVal= 0.1,
#                 farVal=1)

class Camera():
    def __init__(self):
        self.viewMatrix = p.computeViewMatrix(
                cameraEyePosition=[0.5, 0, 0.62],
                cameraTargetPosition=[0.5, 0, 0],
                cameraUpVector=[-1, 0, 0])

        self.projectionMatrix = p.computeProjectionMatrixFOV(
                fov= 65,#math.atan(0.5) * (180/math.pi) * 2 #
                aspect= 4/3,#1.0,
                nearVal= 0.1,
                farVal=1)

        # ------------------ #
        #   Get raw images   #
        # ------------------ #
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                                    width=640, #224 
                                                    height=480,
                                                    viewMatrix=self.viewMatrix,
                                                    projectionMatrix=self.projectionMatrix)

        # ------------------ #
        # Crop the inner bin #
        # ------------------ #
        bin_hsv = cv.cvtColor(rgbImg, cv.COLOR_BGR2HSV)
        image_state = bin_hsv[:, :, 1]
        bin_logic_mask = (image_state <= 200) & (image_state > 100) 
        masked_img = np.uint8(bin_logic_mask) * 255
        
        contours, hierarchy = cv.findContours(masked_img, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
         
        smallest_area = INF
        for i in contours:
            area = cv.contourArea(i)
            if area <= smallest_area:
                smallest_area = area
                inner_bin_contour = i

        rect = cv.minAreaRect(inner_bin_contour)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        self.rect = rect
            
        # final = cv.drawContours(rgbImg,[box],0,(0,255,0),4)
        # cropped_rgbImg = rgbImg[round(rect[0][1]-rect[1][1]/2):round(rect[0][1]+rect[1][1]/2),
        #                        round(rect[0][0]-rect[1][0]/2):round(rect[0][0]+rect[1][0]/2)]
        # ------------------ #
        

    def observe(self):
        # ------------------ #
        #   Get raw images   #
        # ------------------ #
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                                    width=640, #224 
                                                    height=480,
                                                    viewMatrix=self.viewMatrix,
                                                    projectionMatrix=self.projectionMatrix)

        # ------------------ #
        # Crop the inner bin #
        # ------------------ #
        # bin_hsv = cv.cvtColor(rgbImg, cv.COLOR_BGR2HSV)
        # image_state = bin_hsv[:, :, 1]
        # bin_logic_mask = (image_state <= 200) & (image_state > 100) 
        # masked_img = np.uint8(bin_logic_mask) * 255
        
        # contours, hierarchy = cv.findContours(masked_img, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
         
        # smallest_area = INF
        # for i in contours:
        #     area = cv.contourArea(i)
        #     if area <= smallest_area:
        #         smallest_area = area
        #         inner_bin_contour = i

        # rect = cv.minAreaRect(inner_bin_contour)
        # box = cv.boxPoints(rect)
        # box = np.int0(box)
            
        # # final = cv.drawContours(rgbImg,[box],0,(0,255,0),4)
        cropped_rgbImg = rgbImg[round(self.rect[0][1]-self.rect[1][1]/2):round(self.rect[0][1]+self.rect[1][1]/2),
                               round(self.rect[0][0]-self.rect[1][0]/2):round(self.rect[0][0]+self.rect[1][0]/2)]
        # ------------------ #
        
        # ------------------ #
        #   Get state image  #
        # ------------------ #
        obj_hsv = cv.cvtColor(cropped_rgbImg, cv.COLOR_BGR2HSV)
        obj_hsv_s = obj_hsv[:, :, 1]
        obj_logic_mask = obj_hsv_s >= 200
        obj_masked = np.uint8(obj_logic_mask) * 255
        obj_masked_resized = cv.resize(obj_masked, (100, 100))

        # masked_img = depthImg
        # cv.imshow('obj_masked_resized', obj_masked_resized)

        return obj_masked_resized[newaxis,:,:]

    def observe_bin(self):
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                                    width=640, #224 
                                                    height=480,
                                                    viewMatrix=self.viewMatrix,
                                                    projectionMatrix=self.projectionMatrix)
    
        bin_hsv = cv.cvtColor(rgbImg, cv.COLOR_BGR2HSV)
        logic_mask = (bin_hsv[:, :, 1] <= 200) & (bin_hsv[:, :, 1] > 100) 
        bin_masked = np.uint8(logic_mask) * 255
        bin_masked_resized = cv.resize(bin_masked, (100, 100))

        # cv.imshow('bin_masked_resized',bin_masked_resized)

        return bin_masked_resized[newaxis,:,:]



    def observe_obj(self, obj_poses):
        current_obj_poses = obj_poses
        for i in obj_poses:
            # pose = get_pose(i) # This function only works when GUI=True
            pose = p.getBasePositionAndOrientation(i)
            # print(f'\npose({i}) = {pose}\n')
            # if pose[0][0] >= 0:
            current_obj_poses[i] = [list(pose[0]), list(pose[1])]

        return current_obj_poses

    def compare_images(self, img1, img2, threshold=2000):

        if len(img1.shape) == 3:
            img1 = img1[0, :, :]
            img2 = img2[0, :, :]

        errorL2 = cv.norm(img1, img2, cv.NORM_L2)
        print(f'error = {errorL2}')
        if errorL2 <= threshold:
            same = True
        else:
            same = False
        return same


















if __name__ == '__main__':
    connect(use_gui=True)
    scenario = Scenario()
    robot, obj_poses = scenario.scenario()
    scenario.reset_obj_uniform(obj_poses)

    # pushed_dict = pushes(robot, dict)
    camera = Camera()

    # for i in range(1000):
    image1 = camera.observe()



    # image2 = camera.observe_bin()
    
    # ans = camera.compare_images(image1, image2)
    # print(f'ans = \n{ans}\n')
    # cv.imshow('image', image1[0,:,:])
    # plt.imshow(image1[0,:,:])
    # plt.show()
    # cv.imwrite('/home/liu/panda_tamp/src/pddlstream/examples/pybullet/panda/camera_obs.png',image1[0,:,:]*100)
    cv.waitKey(0)

    # print(world_to_pix([0, 0]))
    # print(pushed_dict)















    
