import pybullet as p
import pybullet_utils.bullet_client as bc
import time
import pybullet_data
import os
import cv2 as cv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scenario import Scenario
from examples.pybullet.utils.pybullet_tools.utils import add_data_path, connect, wait_for_user, get_movable_joints, set_joint_positions
from examples.pybullet.utils.pybullet_tools.ikfast.franka_panda.ik import panda_inverse_kinematics_background

class Shadow:
    def __init__(self, num = 8) -> None:
        self.p0 = bc.BulletClient(connection_mode=p.DIRECT) 
        # self.p0 = bc.BulletClient(connection_mode=p.GUI)
        panda_dir = os.path.dirname(os.path.realpath(__file__)) + '/../utils/models/franka_description/robots/'
        self.p0.setAdditionalSearchPath(panda_dir)
        self.startPos = [0,0,0]
        self.startOrientation = self.p0.getQuaternionFromEuler([0,0,0])
        self.panda = self.p0.loadURDF("panda_arm_hand.urdf", self.startPos, self.startOrientation, useFixedBase=True)
        
        # Floor
        self.p0.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p0.loadURDF('plane.urdf')

        # Objects
        dic_list = []
        block_collision_id = self.p0.createCollisionShape(shapeType=self.p0.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
        block_visual_id = self.p0.createVisualShape(shapeType=self.p0.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025], rgbaColor=[0,1,0,1])
        for i in range(num):
            box = self.p0.createMultiBody(baseMass = 0.5,
                                    baseCollisionShapeIndex = block_collision_id,
                                    baseVisualShapeIndex = block_visual_id,
                                    basePosition = [-0.6, 0.0, 0.025])
            # Change the coefficient of friction
            self.p0.changeDynamics(box, -1, lateralFriction=1)
            dic_list.append((box, [-0.6, 0.0, 0.03]))
            
        self.objects_dict = dict(dic_list)
        

        

    def set_joint_angles(self, config):
        # joints = get_movable_joints(self.panda)[:7]
        # print(joints)
        for i in range(7): 
            self.p0.resetJointState(self.panda, i, targetValue=config[i])

    def set_object_pose(self, obj, pose):
        self.p0.resetBasePositionAndOrientation(obj, pose[0], pose[1])



    def blocks_collision_checking(self, robot_config, obj_poses):

        # print(f'\nThe obj_poses in shadow is\n{obj_poses} \n')
        self.set_joint_angles(robot_config)
        ii = 0
        for i in obj_poses:
            pose = obj_poses[i]
            self.set_object_pose(list(self.objects_dict.keys())[ii], pose)

            

            self.p0.performCollisionDetection()
            contact_points = self.p0.getContactPoints(self.panda, list(self.objects_dict.keys())[ii])

            ii += 1


            collision = False
            
            if len(contact_points) > 0:
                collision = True
                break
            else:
                collision = False
        return collision


if __name__ == "__main__":
    connect(use_gui=False)
    scenario = Scenario()
    robot, objects_dict = scenario.scenario()
    collision_checker = Shadow()
    wait_for_user()


    # Position start
    start_point = [0.6, 0.0]
    quat = quaternion_from_euler(0, 3.14, 0, axes='sxyz')
    pose = [start_point+[0.13], quat]
    start_conf = panda_inverse_kinematics_background(robot, 'panda', pose)

    joints = get_movable_joints(robot)
    print(joints)
    arm_joints = joints[:7]
    set_joint_positions(robot, arm_joints, start_conf)
    wait_for_user()
    print(start_conf)
    collision = collision_checker.blocks_collision_checking(start_conf, objects_dict)

    print(collision)
    print(collision_checker.objects_dict)
    print(objects_dict)

    wait_for_user()

