#!/usr/bin/env python

# This file is a substitution of controller

from distutils.log import error
from re import I
import time
import rospy
#import gym
from std_msgs.msg import Float64, Int64, Float64MultiArray
import numpy as np

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose,PoseStamped
import geometry_msgs.msg
import os



from sensor_msgs.msg import JointState


# from spatialmath import SE3
import roboticstoolbox as rtb
# import gym
# import tensorflow as tf
# import swift
# import numpy as np

# from panda_replanning.msg import FollowingPath

class sub_class():
  def __init__(self):
    

    FOLLOWING_POINT_TOPIC = '/moverl/following_point'

    JOINT_STATES_TOPIC = '/joint_states'

    

    self.data_point = Float64MultiArray()

    self.joint_states = JointState()

    self.sub_following_point = rospy.Subscriber(FOLLOWING_POINT_TOPIC, Float64MultiArray, self.sub_following_point_cb)
    self.sub_joint_states = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self.sub_joint_states_cb)
 

  def sub_following_point_cb(self, data):
    self.data_point = data

  def sub_joint_states_cb(self, data):
    self.joint_states = data

 

def distance(array_1, array_2):
  err = np.subtract(array_1, array_2)
  dist = np.linalg.norm(err)
  return dist



class execute_cmd():
  def __init__(self):
    self.joint1_pub = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=10)
    self.joint2_pub = rospy.Publisher('/joint2_position_controller/command', Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher('/joint3_position_controller/command', Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher('/joint4_position_controller/command', Float64, queue_size=10)
    self.joint5_pub = rospy.Publisher('/joint5_position_controller/command', Float64, queue_size=10)
    self.joint6_pub = rospy.Publisher('/joint6_position_controller/command', Float64, queue_size=10)
    self.joint7_pub = rospy.Publisher('/joint7_position_controller/command', Float64, queue_size=10)

    self.cmd_1 = Float64()
    self.cmd_2 = Float64()
    self.cmd_3 = Float64()
    self.cmd_4 = Float64()
    self.cmd_5 = Float64()
    self.cmd_6 = Float64()
    self.cmd_7 = Float64() 

  def execute(self, positions):
    self.cmd_1.data = positions[0]
    self.cmd_2.data = positions[1]
    self.cmd_3.data = positions[2]
    self.cmd_4.data = positions[3]
    self.cmd_5.data = positions[4]
    self.cmd_6.data = positions[5]
    self.cmd_7.data = positions[6]

    self.joint7_pub.publish(self.cmd_7)
    self.joint1_pub.publish(self.cmd_1)
    self.joint2_pub.publish(self.cmd_2)
    self.joint3_pub.publish(self.cmd_3)
    self.joint4_pub.publish(self.cmd_4)
    self.joint5_pub.publish(self.cmd_5)
    self.joint6_pub.publish(self.cmd_6)
    # self.joint7_pub.publish(self.cmd_7)


if __name__ == "__main__":

  rospy.init_node('follow')
  
  # Subscriber
  subscribers = sub_class()
  commander = execute_cmd()

  passed_number_pub = rospy.Publisher('/moverl/passed_number', Int64, queue_size=10)


  r = rospy.Rate(1000)
  r2 = rospy.Rate(100)


  vel = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]) # Angular velocity

  qd0 = vel * 0.00
  qd1 = vel * 0.00

  # robot = rtb.models.DH.Panda()


  point_buffer = []
  n_buffer = []
  old_n = 0

  rospy.sleep(2)
  old_point = np.array(subscribers.joint_states.position[:7])
  time_1 = time.time()
  while not rospy.is_shutdown():
    next_point = np.array(subscribers.data_point.data)


    if len(next_point) == 0:
      print("Wait for the next point...")
      rospy.sleep(0.1)
    
    else: # len(next_point) != 0: # Start receiving points
      new_n = subscribers.data_point.layout.data_offset
      if new_n != old_n: # When a new point comes
        next_point = np.array(subscribers.data_point.data)
        point_buffer.append(next_point)
        n_buffer.append(new_n)
        old_n = new_n
        
      
      
      

      if len(point_buffer) == 0:

        n_buffer = []

      else:
        new_n = subscribers.data_point.layout.data_offset
        if new_n != old_n: # When a new point comes
          next_point = np.array(subscribers.data_point.data)
          point_buffer.append(next_point)
          n_buffer.append(new_n)
          old_n = new_n
          print("renew 1")
        current_point = np.array(subscribers.joint_states.position[:7])
        # current_point = old_point

        # Interpolate the trajectory
        if len(point_buffer) >= 2:
          viapoints = np.array(point_buffer[:2])
          if n_buffer[0] == 1:
            qt = rtb.mstraj(viapoints, dt = 0.01, tacc = 0.0005, qdmax=vel*0.2, q0=current_point)
            del n_buffer[0]
          else:
            print(viapoints)
            qt = rtb.mstraj(viapoints, dt = 0.01, tacc = 0.01, qdmax=vel*0.2, q0=old_point)
        else:
          
          qt = rtb.jtraj(old_point, point_buffer[0], 100)
          first_point = True
 
        # qt = rtb.jtraj(old_point, point_buffer[0], 50)
        q = qt.q
   
  
        

        # Execute the trajectory
        while not rospy.is_shutdown():
          commander.execute(q[0])

          # Renew the buffer
          new_n = subscribers.data_point.layout.data_offset
          if new_n != old_n: # When a new point comes
            next_point = np.array(subscribers.data_point.data)
            point_buffer.append(next_point)
            n_buffer.append(new_n)
            old_n = new_n
            print("renew 2")

          q = np.delete(q, 0, 0)
          r2.sleep()

          if q.shape[0] <= 1:
            old_point = q[0]
            break
          

        # Clean the buffer
        print(f"point_buffer = {len(point_buffer)}")
        
        del point_buffer[0]
          

        # if len(point_buffer) == 0: # Which means the last executed point is the last point
        #   for j in range(q.shape[0]):
        #     commander.execute(q[j])
        

        





      # err = np.subtract(next_point, current_point)
      # err_sign = err/np.abs(err)

      # next = next_point #+  err * 0.001 * vel * 10
      # print(err_sign * vel)
      # next = current_point + err * 0.1

      # if np.amax(np.absolute(err)) <= 0.1:
      #   next = next_point
      # else: 
      #   next = current_point + err * 0.1



    

      
      
        


 
    
    r.sleep()

  # commander.execute([0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.5])