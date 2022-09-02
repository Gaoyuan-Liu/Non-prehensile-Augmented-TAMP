from sre_constants import SUCCESS
import rospy
import os
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
from sensor_msgs.msg import JointState
import numpy as np
import moveit_commander
import tf
# from trac_ik_python.trac_ik import IK
# from handle_gazebo import Gazebo_Handle
# from handle_replan import Replan_Handle
# from handle_scene import Scene_Handle
# from handle_dcsr import DCSR_Handle

from handle_rerrt_franka import RERRT_Handle

from std_msgs.msg import Float64

from franka_gripper.msg import MoveAction, GraspEpsilon, MoveGoal, GraspAction, GraspEpsilon, GraspGoal, GraspActionGoal
import pybullet as p
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from realsense import get_position



class Grasp_Handle:
    
    def __init__(self): 

 
        
        
        self.jointT = JointTrajectory()
        self.jointT.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

        # self.ik_solver = IK("panda_link0", "panda_link8")
        # self.seed_state = [0.0] * self.ik_solver.number_of_joints

        # self.obj_handle = Object_Handle()

        # self.qua_push_x = tf.transformations.quaternion_from_euler(3.14, 0.0, 0.785, 'rxyz')
        # self.qua_push_y = tf.transformations.quaternion_from_euler(3.14, 0.0, -0.785, 'rxyz')

        
  
        self.cmd_finger = Float64()

        self.planner = RERRT_Handle()
        


        # For Franka gripper
        # Gripper state
        self.gripper_states_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_states_sub_cb)
        self.gripper_states = JointState()
        # Releasing parameters
        self.release_client = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
        self.release_client.wait_for_server()
        print('move action client active')
        # Grasping parameters
        self.grasp_client = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
        self.grasp_client.wait_for_server()
        print('grasp action client active')

    def gripper_states_sub_cb(self, data):
        self.gripper_states = data

    def object_location(self, obj_name):
        where_is = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        obj_pose = where_is(obj_name, 'world').pose 
        return obj_pose

    def distance(self, point_1, point_2): 
        err = np.subtract(point_1, point_2)
        dist = np.linalg.norm(err)
        return dist


    def choose_nearest_task(self, task_list, ee_xyz):
        # TODO: 1. list of all the current task
        #       Loop:
        #       2. calculate each task's distance
        #       3. get the index of the smallest one
        #       4. return the task with the index
        # for i in self.task_list_client().model_names:
        #     if "task" in i:
        #         self.task_list.append(i)
        
        dis_min = 10.0
        best_index = 0
        for i in task_list:
            task_pose = self.object_location(i)
            task_xyz = [task_pose.position.x, task_pose.position.y, task_pose.position.z]
            dist = self.distance(task_xyz, ee_xyz)
            if dist < dis_min:
                dis_min = dist
                best_index = task_list.index(i)

        print("Choose " + task_list[best_index] + " to do now.")
        return task_list[best_index]
            

            





    def grasp_configs(self, obj_name):
        obj_pose = self.object_location(obj_name)
        obj_xyz = [obj_pose.position.x, obj_pose.position.y, obj_pose.position.z]

        quaternion = [obj_pose.orientation.x,
                        obj_pose.orientation.y,
                        obj_pose.orientation.z,
                        obj_pose.orientation.w]
        obj_rpy = list(tf.transformations.euler_from_quaternion(quaternion))

        obj_size = [0.05, 0.05, 0.05]

        pose_config_11 = [obj_xyz[0], obj_xyz[1], obj_xyz[2]+0.2] + [3.14, 0.0, np.mod(obj_rpy[-1] - 0.785, np.sign(obj_rpy[-1] - 0.785)*3.14)]
        pose_config_12 = [obj_xyz[0], obj_xyz[1], obj_xyz[2]+0.11] +  [3.14, 0.0, np.mod(obj_rpy[-1] - 0.785, np.sign(obj_rpy[-1] - 0.785)*3.14)]
        pose_config_21 = [obj_xyz[0], obj_xyz[1], obj_xyz[2]+0.2] + [3.14, 0.0, np.mod(obj_rpy[-1] + 0.785, np.sign(obj_rpy[-1] + 0.785)*3.14)]
        pose_config_22 = [obj_xyz[0], obj_xyz[1], obj_xyz[2]+0.11] +  [3.14, 0.0, np.mod(obj_rpy[-1] + 0.785, np.sign(obj_rpy[-1] + 0.785)*3.14)]

        return pose_config_11, pose_config_12, pose_config_21, pose_config_22

   
       

    def open_fingers(self):
        # self.cmd_finger.data = 0.04
        # self.leftfinger_pub.publish(self.cmd_finger)
        # self.rightfinger_pub.publish(self.cmd_finger)
        self.gripper_release()

    def close_fingers(self):
        # self.cmd_finger.data = 0.0
        # self.leftfinger_pub.publish(self.cmd_finger)
        # self.rightfinger_pub.publish(self.cmd_finger)
        self.gripper_grasp()
            

    def grasp(self, obj_name):
        # The grasp contains two parts of motion:
        #   1. Go to the grasp position. for each task, there are two config position, one is above the object,
        #      and the other is the grasping position. 
        #   2. Gripper grasp, a.k.a. fingers close. 

        # Motion 1
        self.open_fingers()
        pose_config_11, pose_config_12, pose_config_21, pose_config_22 = self.grasp_configs(obj_name)
        try:
            go_success_1, replan_time_1 = self.planner.go(pose_config_11)
            if go_success_1 == True:
                go_success_2, replan_time_2 = self.planner.go(pose_config_12)
                # rospy.sleep(0.5)
                # go_success_2 = self.planner.go_down()
                # replan_time_2 = 0
            else:
                go_success_2, replan_time_2 = False, 0

            if go_success_2 == False:
                print("try again")
                go_success_1, replan_time_1 = self.planner.go(pose_config_21)
                if go_success_1 == True:
                    go_success_2, replan_time_2 = self.planner.go(pose_config_22)
                    # rospy.sleep(0.5)
                    # go_success_2 = self.planner.go_down()
                    # replan_time_2 = 0
                else:
                    go_success_2, replan_time_2 = False, 0

        except:
            print("Cannot go to the grasping position.")
            go_success_1, replan_time_1 = False, 0
            go_success_2, replan_time_2 = False, 0

        go_success = go_success_1 and go_success_2
        grasp_replan_time = replan_time_1 + replan_time_2

        # Motion 2
        if go_success == True:
            self.close_fingers()
            rospy.sleep(1.0)
            # Use the fingers distance to determin grasp success
            if self.gripper_states.position[1] <= 0.03:
                grasp_success = True
            # else:
            #     grasp_success = False
        else:
            grasp_success = False



        return go_success, grasp_success, grasp_replan_time

    def drop(self):
        drop_replan_time = 0
        # self.planner.go_up()
        go_success, drop_replan_time = self.planner.go([-0.5, 0.0, 0.5, 3.14, 0, 0])
        return go_success, drop_replan_time


    def task(self, task_name):
        task_success = False

        # Grasp
        go_success, grasp_success, grasp_replan_time = self.grasp(task_name)
        
        # Drop
        drop_success = False
        drop_replan_time = 0
        if grasp_success == True:
            # self.scene.update()
            # self.scene.attach_task(task_name)
            drop_success, drop_replan_time = self.drop()

            if drop_success == True:
                task_success = True
                print("Task finished!")
            
        task_replan_time = grasp_replan_time + drop_replan_time

        # self.open_fingers()
        # self.scene.detach_task(task_name)
        rospy.sleep(0.5)

        return go_success, task_success, task_replan_time

   
    def gripper_release(self, width=0.8):
        print("Releasing object")
        joint_states = [width/2, width/2]
        goal = MoveGoal(width=float(width), speed=float(0.1))
        self.release_client.send_goal(goal)
        self.release_client.wait_for_result(rospy.Duration.from_sec(5.0))


    def gripper_grasp(self, width=0.2):
        print("Grasping object")
        GRASP_EPSILON = GraspEpsilon(inner=0.039, outer=0.039)
        goal = GraspGoal(width=float(0.04), epsilon=GRASP_EPSILON, speed=float(0.1), force=float(100))
        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result(rospy.Duration.from_sec(5.0))
    



      


if __name__ == "__main__":

    outdir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(outdir)
    rospy.init_node("handle_grasp")

    grasp = Grasp_Handle()
    planner = grasp.planner
    move_group = planner.move_group

    ######################
    #    Open fingers    #
    ######################
    
    grasp.gripper_release(0.8)

    ######################
    #     Go initial     #
    ######################

    quat = quaternion_from_euler(3.14/4, 3.14, 0,'szxy')
    move_group.set_pose_target([0.5, 0.0, 0.5] + list(quat))
    move_group.plan()
    move_group.go()
    grasp.gripper_release(0.8)

    ######################
    #       Observe      #
    ######################

    obj_dict = get_position()
    bias = [0.03, -0.04]

    ######################
    #   Start grasping   #
    ######################
    for i in obj_dict:
    
        ######################
        #  Go to the object  #
        ######################
        quat = quaternion_from_euler(3.14/4 + obj_dict[i][2]/57.3, 3.14, 0,'szxy')
        move_group.set_pose_target([0.5+obj_dict[i][0]+bias[0], 0.0+obj_dict[i][1]+bias[1], 0.2] + list(quat))
        move_group.plan()
        move_group.go()


        ######################
        #       Go dwon      #
        ######################
        planner.go_down()

        
        ######################
        #    Close fingers   #
        ######################
        grasp.gripper_grasp()



        ######################
        #       Go up        #
        ######################
        planner.go_up()

        ######################
        #      Release       #
        ######################
        quat = quaternion_from_euler(3.14/4, 3.14, 0,'szxy')
        move_group.set_pose_target([0.4, 0.4, 0.3] + list(quat))
        move_group.plan()
        move_group.go()
        grasp.gripper_release(0.8)












    




        
        
    