#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray, Bool

from examples.pybullet.utils.pybullet_tools.panda_primitives import Trajectory, GripperCommand, Pose, Conf, get_ik_ir_gen, get_motion_gen, get_stable_gen, get_grasp_gen, control_commands

dumb_t = Trajectory([0.0])
dumb_GripperCommand = GripperCommand


TYPE_t = type(dumb_t)

class ROS_Commander:

    def __init__(self, commands):
        self.commands = commands
        self.tpublisher = rospy.Publisher('/moverl/following_point', Float64MultiArray, queue_size=10)
        self.next_point = Float64MultiArray()
        self.cmd_finger = Float64()
        self.leftfinger_pub = rospy.Publisher('/panda/joint_leftfinger_position_controller/command', Float64, queue_size=10)
        self.rightfinger_pub = rospy.Publisher('/panda/joint_rightfinger_position_controller/command', Float64, queue_size=10)
        rospy.sleep(2)
        

    def t_execution(self, t):
        data_n = 0
        for conf in t.path:
            data_n += 1
            self.next_point.data = conf.values
            self.next_point.layout.data_offset = data_n
            self.tpublisher.publish(self.next_point)
            rospy.sleep(0.01)

    def GripperCommand_execution(self, GripperCommand):
        self.cmd_finger.data = GripperCommand.position
        self.leftfinger_pub.publish(self.cmd_finger)
        self.rightfinger_pub.publish(self.cmd_finger)

    def Attach_execution(self):
        a = 1


    def ros_commander(self):
        
        for c in self.commands:
            # try:
            if c.type == 'Trajectory':
                self.t_execution(c)
                # if c.type == 'GripperCommand':
                #     self.GripperCommand_execution(c)

            # except:
            #     print("No such type.")




#######################################################

# def main(verbose=True):
    

# if __name__ == '__main__':
#     main()