#!/usr/bin/env python

import imp
from multiprocessing.reduction import ForkingPickler
import rospy
from std_msgs.msg import Float64, Float64MultiArray, Bool

from examples.pybullet.utils.pybullet_tools.panda_primitives import Trajectory, GripperCommand, Pose, Conf, get_ik_ir_gen, get_motion_gen, get_stable_gen, get_grasp_gen, control_commands
from franka_gripper.msg import MoveAction, GraspEpsilon, MoveGoal, GraspAction, GraspGoal, GraspActionGoal
import actionlib
from sensor_msgs.msg import JointState
import moveit_commander
import numpy as np

dumb_t = Trajectory([0.0])
dumb_GripperCommand = GripperCommand


TYPE_t = type(dumb_t)

class ROS_Commander:

    def __init__(self, commands):
        self.commands = commands
        self.tpublisher = rospy.Publisher('/moverl/following_point', Float64MultiArray, queue_size=10)
        self.next_point = Float64MultiArray()
        
        # Moveit
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        

        
        
        # For grasping
        # Gripper state
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_sub_cb)
        self.joint_states = JointState()
        # Releasing parameters
        self.release_client = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
        self.release_client.wait_for_server()
        print('move action client active')
        # Grasping parameters
        self.grasp_client = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
        self.grasp_client.wait_for_server()
        print('grasp action client active')
        # rospy.sleep(2)


    def joint_states_sub_cb(self, data):
        self.joint_states = data


    def gripper_release(self, width=0.8):
        print("Releasing object")
        joint_states = [width/2, width/2]
        goal = MoveGoal(width=float(width), speed=float(0.1))
        self.release_client.send_goal(goal)
        self.release_client.wait_for_result(rospy.Duration.from_sec(5.0))


    def gripper_grasp(self):
        print("Grasping object")
        GRASP_EPSILON = GraspEpsilon(inner=0.039, outer=0.039)
        goal = GraspGoal(width=float(0.04), epsilon=GRASP_EPSILON, speed=float(0.1), force=float(100))
        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result(rospy.Duration.from_sec(5.0))


        

    def t_execution(self, t):
        data_n = 0
        for conf in t.path:
            data_n += 1
            self.next_point.data = conf.values
            self.next_point.layout.data_offset = data_n
            self.tpublisher.publish(self.next_point)
            rospy.sleep(0.05)
        while True:
            dis = np.linalg.norm(np.array(self.joint_states.position[:7]) - t.path[-1].values)
            rospy.sleep(0.05)
            # print(dis)
            if dis <= 0.1:
                break

    def GripperCommand_execution(self, GripperCommand):
        # self.cmd_finger.data = GripperCommand.position
        # self.leftfinger_pub.publish(self.cmd_finger)
        # self.rightfinger_pub.publish(self.cmd_finger)
        print(GripperCommand.position)
        if GripperCommand.position <= 0.03:
            self.gripper_grasp()
            rospy.sleep(1)
        else:
            self.gripper_release()
            rospy.sleep(1)



    def Attach_execution(self):
        a = 1


    def reset_joint(self):
        
        self.move_group.set_joint_value_target([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0])
        plan_tuple = self.move_group.plan()
        data_n = 0
        for waypoint in plan_tuple[1].joint_trajectory.points:
            data_n += 1
            self.next_point.data = waypoint.positions
            self.next_point.layout.data_offset = data_n
            self.tpublisher.publish(self.next_point)
            rospy.sleep(0.05)




    def ros_commander(self):

        self.reset_joint()
        
        for c in self.commands:
            # try:
            if c.type == 'Trajectory':
                self.t_execution(c)
            if c.type == 'GripperCommand':
                self.GripperCommand_execution(c)

            # except:
            #     print("No such type.")




#######################################################

# def main(verbose=True):
    

# if __name__ == '__main__':
#     main()