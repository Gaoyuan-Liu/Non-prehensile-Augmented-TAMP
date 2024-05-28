from __future__ import print_function
from turtle import pos


import numpy as np
import time
import os
import sys
from matplotlib import pyplot as plt
import math

file_path = os.path.dirname(os.path.realpath(__file__))

# sys.path.insert(0, file_path + '/../')

import pybullet as p


# from moveit_msgs.msg import RobotState
# from geometry_msgs.msg import Pose
import cv2 as cv


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
         
        smallest_area = np.inf
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
    pass















    
