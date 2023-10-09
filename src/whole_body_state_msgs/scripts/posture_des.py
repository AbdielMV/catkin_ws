#!/usr/bin/env python
import rospy
import pinocchio
from std_msgs.msg import String
from whole_body_state_msgs.msg import WholeBodyState

from geometry_msgs.msg import Accel

import numpy as np

urdf_filename = '/home/niger/reemc_public_ws/src/whole_body_state_msgs/urdf/reemc_full_ft_hey5.urdf'
free_flyer = pinocchio.JointModelFreeFlyer()

model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
data = model.createData()

q = np.zeros(model.nq)
v = np.zeros(model.nv)
tau = np.zeros(model.njoints - 2)

def robot_states(msg):

    q[3] = msg.centroidal.base_orientation.x
    q[4] = msg.centroidal.base_orientation.y
    q[5] = msg.centroidal.base_orientation.z
    q[6] = msg.centroidal.base_orientation.w
    v[3] = msg.centroidal.base_angular_velocity.x
    v[4] = msg.centroidal.base_angular_velocity.y
    v[5] = msg.centroidal.base_angular_velocity.z
    
    for j in range(len(msg.joints)):
        jointId = model.getJointId(msg.joints[j].name) - 2
        q[jointId + 7] = msg.joints[j].position
        v[jointId + 6] = msg.joints[j].velocity
        tau[jointId] = msg.joints[j].effort

    pinocchio.centerOfMass(model, data, q, v)
    q[0] = msg.centroidal.com_position.x - data.com[0][0]
    q[1] = msg.centroidal.com_position.y - data.com[0][1]
    q[2] = msg.centroidal.com_position.z - data.com[0][2]
    v[0] = msg.centroidal.com_velocity.x - data.vcom[0][0]
    v[1] = msg.centroidal.com_velocity.y - data.vcom[0][1]
    v[2] = msg.centroidal.com_velocity.z - data.vcom[0][2]

def talker():
   rospy.init_node('posture_des') #anonymous=True
   pub = rospy.Publisher('chatter', String, queue_size=10)
   posture_des_pub = rospy.Publisher('posture_des', Accel, queue_size=10)
   rospy.Subscriber("robot_states", WholeBodyState, robot_states)
   rate = rospy.Rate(10) # 10hz

   q_des = [0, 0, 0, 0, 0, 0]
   kp = 4
   kd = 6
   while not rospy.is_shutdown():
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      posture_des_task = Accel()
      posture_des_task.linear.x = kp*(q_des[0] - q[22]) - kd*v[21] 
      posture_des_task.linear.y = kp*(q_des[1] - q[23]) - kd*v[22] 
      posture_des_task.linear.z = kp*(q_des[2] - q[24]) - kd*v[23] # 
      posture_des_pub.publish(posture_des_task)
      rate.sleep()

if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
      
