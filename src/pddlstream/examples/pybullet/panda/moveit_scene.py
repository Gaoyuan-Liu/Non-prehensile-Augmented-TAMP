#! /venv3.8/bin/python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
import math
import time
import copy
import threading
from gazebo_msgs.srv import GetModelProperties, GetModelState, GetWorldProperties
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import tf
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from realsense import Camera


class Moveit_Scene:
    
    def __init__(self):
        #self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.scene = moveit_commander.PlanningSceneInterface()
       

        rospy.sleep(1.0)
        self.obs_size_cylinder = np.array([0.5, 0.08]) # [0.4, 0.05] [length, radius]
        self.obs_size_box = np.array([0.1, 0.1, 0.1])

        # Table
        obs_pose = PoseStamped()
        obs_pose.header.frame_id = 'world'
        obs_pose.pose.position.z = -1.001
        self.scene.add_box('table', obs_pose, size=(2, 2, 0.002))

        # Bin
        # bin_center = [0.5, 0.0]
        # bin_size = [0.27, 0.39, 0.05]
        # bin_thickness = 0.005


        # obs_pose = PoseStamped()
        # obs_pose.header.frame_id = 'world'

        # obs_pose.pose.position.x = bin_center[0]+bin_size[0]/2-bin_thickness/2
        # obs_pose.pose.position.y = bin_center[1]
        # obs_pose.pose.position.z = bin_size[2]/2
        # self.scene.add_box('bin1', obs_pose, size=(bin_thickness, bin_size[1], bin_size[2]))

        # obs_pose.pose.position.x = bin_center[0]-bin_size[0]/2+bin_thickness/2
        # obs_pose.pose.position.y = bin_center[1]
        # obs_pose.pose.position.z = bin_size[2]/2
        # self.scene.add_box('bin2', obs_pose, size=(bin_thickness, bin_size[1], bin_size[2]))

        # obs_pose.pose.position.x = bin_center[0]
        # obs_pose.pose.position.y = bin_center[1]+bin_size[1]/2-bin_thickness/2
        # obs_pose.pose.position.z = bin_size[2]/2
        # self.scene.add_box('bin3', obs_pose, size=(bin_size[0], bin_thickness, bin_size[2]))

        # obs_pose.pose.position.x = bin_center[0]
        # obs_pose.pose.position.y = bin_center[1]-bin_size[1]/2+bin_thickness/2
        # obs_pose.pose.position.z = bin_size[2]/2
        # self.scene.add_box('bin4', obs_pose, size=(bin_size[0], bin_thickness, bin_size[2]))




        


        

    def add_objects_collision(self, obj_poses):
        for i in obj_poses:
            obj_name = str(i)
            xyyaw = obj_poses[i]
            obj_pose = PoseStamped()
            obj_pose.header.frame_id = 'world'
            obj_pose.pose.position.x = xyyaw[0]
            obj_pose.pose.position.y = xyyaw[1]
            obj_pose.pose.position.z = 0.05
            quat = quaternion_from_euler(xyyaw[2], 0, 0,'szxy')
            obj_pose.pose.orientation.x = quat[0]
            obj_pose.pose.orientation.y = quat[1]
            obj_pose.pose.orientation.z = quat[2]
            obj_pose.pose.orientation.w = quat[3]
            self.scene.add_box(obj_name, obj_pose, size=(0.06, 0.06, 0.05))

    def remove_objects_collision(self, obj_poses):
        for i in obj_poses:
            self.scene.remove_world_object(str(i))




        

if __name__ == "__main__":
    # Locate local working directory on where this file is
    outdir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(outdir)

    rospy.init_node("handle_scene")
    camera = Camera()
    obj_poses = camera.get_positions()
    scene = Moveit_Scene()

    scene.add_objects_collision(obj_poses)

    time.sleep(3)

    scene.remove_objects_collision(obj_poses)




  
  
    
   
    






    




        
        
    