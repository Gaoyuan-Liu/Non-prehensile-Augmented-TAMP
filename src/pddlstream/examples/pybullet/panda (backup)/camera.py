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

viewMatrix = p.computeViewMatrix(
                cameraEyePosition=[0.5, 0, 1],
                cameraTargetPosition=[0.5, 0, 0],
                cameraUpVector=[1, 0, 0])

projectionMatrix = p.computeProjectionMatrixFOV(
                fov=math.atan(0.5) * (180/math.pi) * 2,
                aspect=1.0,
                nearVal=0.1,
                farVal=1)

def observe():
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
                                                width=100, #224 
                                                height=100,
                                                viewMatrix=viewMatrix,
                                                projectionMatrix=projectionMatrix)
  
    image_hsv = cv2.cvtColor(rgbImg, cv2.COLOR_BGR2HSV)
    
    image_state = image_hsv[:, :, 1]
  
    return image_state

def world_to_pix(real_position):
    x_pix = math.trunc((1 - real_position[0]) *100)

    y_pix = math.trunc((real_position[1]+0.5) *100)

    return [x_pix, y_pix]











if __name__ == '__main__':
    connect(use_gui=True)
    robot, dict = scenario()
    # pushed_dict = pushes(robot, dict)
    camera()

    print(world_to_pix([0, 0]))
    # print(pushed_dict)















    # # Show the pybullet window
    # connect(use_gui=True)

    # add_data_path()
    # floor = load_pybullet("plane.urdf")


    # panda = create_panda() # Here panda is a int(1)

    # joints = get_movable_joints(panda)
    # arm_joints = joints[:7]
    
    # initial_conf = (0.0, 0.0, 0.0, -0.5, 0.0, 0.5, 0.0, 0.02, 0.02)
    # print(initial_conf)
    
    # arm = 'panda'
    # set_arm_conf(panda, joints, initial_conf)
    # close_arm(panda, arm)
    
    

    # # Block
    # block_width = 0.05
    # block_height = 0.05

    

    # collision_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
    
    # visual_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])

    
    # box = p.createMultiBody(baseMass = 0.1,
    #                         baseCollisionShapeIndex = collision_id,
    #                         baseVisualShapeIndex = visual_id,
    #                         basePosition = [0.4, -0.2, 0.03])
    # box2 = p.createMultiBody(baseMass = 0.1,
    #                         baseCollisionShapeIndex = collision_id,
    #                         baseVisualShapeIndex = visual_id,
    #                         basePosition = [0.4, -0.1, 0.03])

    





    # # p.setRealTimeSimulation(1)
    # p.setGravity(0,0,-10)
    
    # rospy.init_node('pushing')
    # rospy.sleep(2)

    # # Moveit
    # move_group = moveit_commander.MoveGroupCommander("panda_arm")
    # move_group.set_planner_id('RRTstarkConfigDefault')



    # push([0.4, 0.5], [0.4, -0.1])
    

















    # # Position start
    # quat = quaternion_from_euler(0, 3.14, 0, axes='sxyz')
    # pose = [[0.4, -0.2, 0.15], quat]
    # start_conf = panda_inverse_kinematics_background(panda, arm, pose)
    # robot_state = RobotState()
    # robot_state.joint_state.name = move_group.get_joints()[:7]
    # robot_state.joint_state.position = start_conf

    # set_joint_positions(panda, arm_joints, start_conf)
    # move_group.set_start_state(robot_state)
   

    # # Position end
    # quat = quaternion_from_euler(0, 3.14, -3.14/4, axes='sxyz')
    # end_pose = Pose()
    # end_pose.position.x = 0.5
    # end_pose.position.y = 0.0
    # end_pose.position.z = 0.15
    # end_pose.orientation.x = quat[0]
    # end_pose.orientation.y = quat[1]
    # end_pose.orientation.z = quat[2]
    # end_pose.orientation.w = quat[3]


    # # Set Cartesian path
    # waypoints = [end_pose]

    # (plan_waypoints, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    # # path = move_group.plan()
    # # joint_trajectory:
    # # for point in path[1].joint_trajectory.points:
    # # for point in plan_waypoints.joint_trajectory.points:
    # #     p.setJointMotorControlArray(bodyIndex=panda, # bodyUniqueId
    # #                                jointIndices=arm_joints,
    # #                                controlMode=p.POSITION_CONTROL,
    # #                                targetPositions=point.positions,
    # #                                targetVelocities=[0.0] * 7,
    # #                                forces=[100.0] * 7
    # #                                )
    # #     time.sleep(0.01)
        
    
    # for point in plan_waypoints.joint_trajectory.points:
    #     for _ in joint_controller_hold(panda, arm_joints, point.positions):
    #             step_simulation()






    #             # time.sleep(0.01)
    #     # control_joints(body=panda, joints=arm_joints, positions=point.positions)
    #     # positions = get_joint_positions(body=panda, joints=arm_joints)
    #     # while not np.allclose(positions, point.positions, atol=1e-3, rtol=0):
    #         # time_elapsed += dt
    #         # positions = get_joint_positions(body=panda, joints=arm_joints)
        
    #     # joint_controller(body=panda, joints=arm_joints, target=point.positions)
    #     # print('hello')
    #     # time.sleep(0.1)

    

    # # move_group.set_start_state()
    

    # wait_for_user()