#!/usr/bin/env python

import rospy
import pinocchio
from std_msgs.msg import String
from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

import numpy as np

urdf_filename = '/home/abdiel/catkin_ws/src/whole_body_state_msgs/urdf/reemc_full_ft_hey5.urdf'
free_flyer = pinocchio.JointModelFreeFlyer()

topic = 'robot_states'
model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
frame_id = "base_link" 

wbs = WholeBodyStatePublisher(topic, model, frame_id)

q = np.zeros(model.nq)
v = np.zeros(model.nv)
tau = np.zeros(model.njoints - 2)

wrench_right_force =  np.zeros(3)
wrench_right_torque =  np.zeros(3)

wrench_left_force =  np.zeros(3)
wrench_left_torque =  np.zeros(3)

right_nsurf = [0, 0, 4]
right_frict_coeff = 3

left_nsurf = [0, 0, 4]
left_frict_coeff = 3




def floating_base_callback(msg):
   q[0] = msg.pose.pose.position.x
   q[1] = msg.pose.pose.position.y
   q[2] = msg.pose.pose.position.z
   q[3] = msg.pose.pose.orientation.x
   q[4] = msg.pose.pose.orientation.y
   q[5] = msg.pose.pose.orientation.z
   q[6] = msg.pose.pose.orientation.w
    
   v[0] =  msg.twist.twist.linear.x
   v[1] =  msg.twist.twist.linear.y
   v[2] =  msg.twist.twist.linear.z
   v[3] =  msg.twist.twist.angular.x
   v[4] =  msg.twist.twist.angular.y
   v[5] =  msg.twist.twist.angular.z

def joint_state_callback(msg):
   '''for i in range(7,21):
      q[i] = msg.position[i+16-7]
   for i in range(0,17):
      q[i+13+7] = msg.position[i]

   for i in range(6,20):
      v[i] = msg.velocity[i+16-6]
   for i in range(0,17):
      v[i+13+6] = msg.velocity[i]'''

   for i in range(0,14):
      q[i+7] = msg.position[i+16]
   for i in range(0,16):
      q[i+14+7] = msg.position[i]

   for i in range(0,14):
      v[i+6] = msg.velocity[i+16]
   for i in range(0,16):
      v[i+14+6] = msg.velocity[i]

   for i in range(0,14):
      tau[i] = msg.effort[i+16]
   for i in range(0,16):
      tau[i+14] = msg.effort[i]

def wrench_right_callback(msg):
   wrench_right_force[0] = msg.wrench.force.x
   wrench_right_force[1] = msg.wrench.force.y
   wrench_right_force[2] = msg.wrench.force.z

   wrench_right_torque[0] = msg.wrench.torque.x
   wrench_right_torque[1] = msg.wrench.torque.y
   wrench_right_torque[2] = msg.wrench.torque.z

def wrench_left_callback(msg):
   wrench_left_force[0] = msg.wrench.force.x
   wrench_left_force[1] = msg.wrench.force.y
   wrench_left_force[2] = msg.wrench.force.z

   wrench_left_torque[0] = msg.wrench.torque.x
   wrench_left_torque[1] = msg.wrench.torque.y
   wrench_left_torque[2] = msg.wrench.torque.z

class WrenchRightAnkle:
   linear = wrench_right_force   
   angular = wrench_right_torque

class WrenchLeftAnkle:
   linear = wrench_left_force   
   angular = wrench_left_torque
p=dict()
pd=dict()
s = {'left_sole_link':[left_nsurf, left_frict_coeff], 'right_sole_link':[right_nsurf, right_frict_coeff]}
f = {'left_sole_link':[0, WrenchLeftAnkle()], 'right_sole_link':[0, WrenchRightAnkle()]}

#s = {'right_sole_link':[right_nsurf, right_frict_coeff], 'left_sole_link':[left_nsurf, left_frict_coeff]}
#f = {'right_sole_link':[0, WrenchRightAnkle()], 'left_sole_link':[0, WrenchLeftAnkle()]}

def talker():
   rospy.init_node('wbs') #anonymous=True
   pub = rospy.Publisher('chatter', String, queue_size=10)
   rospy.Subscriber('/floating_base_pose_simulated', Odometry, floating_base_callback, queue_size=10)
   rospy.Subscriber('/joint_states', JointState, joint_state_callback, queue_size=10)
   rospy.Subscriber('/right_ankle_ft', WrenchStamped, wrench_right_callback, queue_size=10)
   rospy.Subscriber('/left_ankle_ft', WrenchStamped, wrench_left_callback, queue_size=10)
   wbs._pub
   wbs._wb_iface
#   x = wbs._wb_iface._model.njoints - 2
#   print(x)
#   print(wbs._wb_iface._msg.joints)
   rate = rospy.Rate(10000) # 10hz


   while not rospy.is_shutdown():
      t =  rospy.Time.now().secs
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      wbs.publish(t, q, v, tau, p, pd, f, s)
      rate.sleep()

if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
      
