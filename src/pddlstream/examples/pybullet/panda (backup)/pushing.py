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


def pushing_scenario(grasp_type='top', num=4):
    # Show the pybullet window
    connect(use_gui=True)

    add_data_path()
    floor = load_pybullet("plane.urdf")


    panda = create_panda() # Here panda is a int(1)

    joints = get_movable_joints(panda)
    arm_joints = joints[:7]
    

    
    initial_conf = (0.0, 0.0, 0.0, -0.5, 0.0, 0.5, 0.0, 0.02, 0.02)
   
    
    arm = 'panda'
    set_arm_conf(panda, joints, initial_conf)
    open_arm(panda, arm)
    

    # Block
    block_width = 0.05
    block_height = 0.05
    num = num
    blocks = [create_box(block_width, block_width, block_height, mass = 0.01, color=GREEN) for _ in range(num)]
    grasp_type = grasp_type
    # min_distances = {block: 0.05 for block in blocks}
    # initial_surfaces = {block: [table, floor] for block in blocks}
    # sample_placements(initial_surfaces, min_distances=min_distances)
    set_point(blocks[0], Point(x=10.6, y=0.0, z=block_height/2 ))
    set_point(blocks[1], Point(x=10.6, y=0.06, z=block_height/2 ))
    set_point(blocks[2], Point(x=10.6, y=-0.06, z=block_height/2))
    # set_point(blocks[2], Point(x=0.6-0.06, y=0.0, z=block_height/2 ))
    # # set_euler(blocks[2], Euler(roll=0, pitch=0, yaw=3.14/2))
    set_point(blocks[3], Point(x=10.6+0.06, y=0.0, z=block_height/2))

    

   

    p.setRealTimeSimulation(1)
    
   


def push(robot, move_group, start_point, end_point): # 2D list

    joints = get_movable_joints(robot)
    arm_joints = joints[:7]

    # Calculate orientation
    yaw = angle_between([1,0], np.subtract(end_point, start_point))
    print('yaw = ' + str(yaw) )
    if start_point[1] > end_point[1]:
        yaw = -yaw

    # Position start
    quat = quaternion_from_euler(0, 3.14, yaw, axes='sxyz')
    pose = [start_point+[0.15], quat]
    start_conf = panda_inverse_kinematics_background(robot, 'panda', pose)
    robot_state = RobotState()
    robot_state.joint_state.name = move_group.get_joints()[:7]
    robot_state.joint_state.position = start_conf

    set_joint_positions(robot, arm_joints, start_conf)
    move_group.set_start_state(robot_state)

    quat = quaternion_from_euler(0, 3.14, yaw-3.14/4, axes='sxyz')
    start_pose = Pose()
    start_pose.position.x = (end_point[0] + start_point[0])/2
    start_pose.position.y = (end_point[1] + start_point[1])/2
    start_pose.position.z = 0.15
    start_pose.orientation.x = quat[0]
    start_pose.orientation.y = quat[1]
    start_pose.orientation.z = quat[2]
    start_pose.orientation.w = quat[3]

    # Position end
    quat = quaternion_from_euler(0, 3.14, yaw-3.14/4, axes='sxyz')
    end_pose = Pose()
    end_pose.position.x = end_point[0]
    end_pose.position.y = end_point[1]
    end_pose.position.z = 0.15
    end_pose.orientation.x = quat[0]
    end_pose.orientation.y = quat[1]
    end_pose.orientation.z = quat[2]
    end_pose.orientation.w = quat[3]

    # Set Cartesian path
    waypoints = [start_pose, end_pose]
    (plan_waypoints, fraction) = move_group.compute_cartesian_path(waypoints, eef_step = 0.01, jump_threshold=5.0)

    for point in plan_waypoints.joint_trajectory.points:
        for _ in joint_controller_hold(robot, arm_joints, point.positions, timeout=1 ):
            step_simulation()
            # time.sleep(0.001)

    for i in range(100):
        step_simulation()









def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    # print(f'vector = {vector}')
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def scenario(num=3):
    p.setGravity(0,0,-10)
    add_data_path()
    floor = load_pybullet("plane.urdf")
    
    # Robot
    panda = create_panda()
    robot = panda
    
    # Objects
    dic_list = []
    block_collision_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
    block_visual_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025], rgbaColor=[0,1,0,1])

    for i in range(num):
        # dic_list.append((i + 1, np.zeros((2, 3), dtype=np.float64)))

        box = p.createMultiBody(baseMass = 0.1,
                                baseCollisionShapeIndex = block_collision_id,
                                baseVisualShapeIndex = block_visual_id,
                                basePosition = [0.6, 0.0, 0.03])
        dic_list.append((box, [0.6, 0.0, 0.03]))

    p.resetBasePositionAndOrientation(dic_list[0][0], [0.6, 0.0, 0.03], [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(dic_list[1][0], [0.6, -0.06, 0.03], [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(dic_list[2][0], [0.6-0.06, 0.0, 0.03], [0, 0, 0, 1])
    # p.resetBasePositionAndOrientation(dic_list[3][0], [1, 0.5, 0.03], [0, 0, 0, 1])

    objects_dict = dict(dic_list)

    # Wall
    # wall_collision_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.1, 0.01, 0.1])
    # wall_visual_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.1, 0.01, 0.1], rgbaColor=[0.19,0.19,0.19,1])
    # wall = p.createMultiBody(baseMass = 0,
    #                             baseCollisionShapeIndex = wall_collision_id,
    #                             baseVisualShapeIndex = wall_visual_id,
    #                             basePosition = [0.6, -0.2, 0.05])


    return robot, objects_dict

def camera():
    # Camera parameters
    viewMatrix = p.computeViewMatrix(
                    cameraEyePosition=[0.5, 0, 1],
                    cameraTargetPosition=[0.5, 0, 0],
                    cameraUpVector=[1, 0, 0])

    projectionMatrix = p.computeProjectionMatrixFOV(
                    fov=math.atan(0.5) * (180/math.pi) * 2,
                    aspect=1.0,
                    nearVal=0.1,
                    farVal=1)

    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                                width=100, #224 
                                                height=100,
                                                viewMatrix=viewMatrix,
                                                projectionMatrix=projectionMatrix)
    # print(rgbImg)

    plt.imshow(rgbImg)
    plt.show()
    image_hsv = cv2.cvtColor(rgbImg, cv2.COLOR_BGR2HSV)
    # print(image_hsv.shape)
    plt.imshow(image_hsv[:, :, 1])
    plt.colorbar()
    plt.show()
    # plt.savefig("camera_example.png", bbox_inches="tight")
    image_state = image_hsv[:, :, 1]
    # print(image_state.shape)
    # print(math.atan(0.5))
    return image_state

def world_to_pix(real_position):
    x_pix = math.trunc(real_position[0] *100)

    y_pix = math.trunc((real_position[1]+0.5) *100)

    return [x_pix, y_pix]





def pushes(robot, objects_dict):

    # panda = create_panda() # Here panda is a int(1)

    joints = get_movable_joints(robot)
    arm_joints = joints[:7]
    
    initial_conf = (0.0, 0.0, 0.0, -0.5, 0.0, 0.5, 0.0, 0.02, 0.02)
    
    arm = 'panda'
    set_arm_conf(robot, joints, initial_conf)
    close_arm(robot, arm)
    
    rospy.init_node('pushing')
    rospy.sleep(2)

    # Moveit
    move_group = moveit_commander.MoveGroupCommander("panda_arm")
    move_group.set_planner_id('RRTstarkConfigDefault')

    # Push
    push(robot, move_group, [0.6, 0.2], [0.6, -0.1])

    # Get obj position after pushing
    for i in objects_dict:
        pose = get_pose(i)
        objects_dict[i][0] = pose[0][0]
        objects_dict[i][1] = pose[0][1]
        objects_dict[i][2] = pose[0][2]
    
    return objects_dict






if __name__ == '__main__':
    connect(use_gui=True)
    robot, dict = scenario()
    # pushed_dict = pushes(robot, dict)
    map = camera()
    
    # pix1 = world_to_pix([60, 55])
    # pix2 = world_to_pix([55, 60])

    # print(map[pix1[0], pix1[1]])
    # print(map[pix2[0], pix2[1]])
    print('hello')
    # print(map[36, 48])
    print(map[0, 0])
    # print(world_to_pix([0, 0]))
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