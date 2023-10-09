#!/usr/bin/env python
import rospy
import pinocchio
from std_msgs.msg import String
from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher
from whole_body_state_msgs.msg import WholeBodyState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
import numpy as np

urdf_filename = '/home/niger/reemc_public_ws/src/reemc_simulation/reemc_gazebo/models/reemc_full/reemc_full_ft_hey5.urdf.urdf'
free_flyer = pinocchio.JointModelFreeFlyer()
model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
data = model.createData()

q = np.zeros(model.nq)
v = np.zeros(model.nv)
tau = np.zeros(model.njoints - 2)

joint=""

x_pos=0
y_pos=0
z_pos=0

x_vel=0
y_vel=0
z_vel=0

x_acc=0
y_acc=0
z_acc=0

Pkkr_right=[0,0,0]
Pkkr_left=[0,0,0]
Tkkkr_right=[0,0,0,0,0,0]
Tkkkr_left=[0,0,0,0,0,0]
Tkkkr_dot_right=[0,0,0,0,0,0]
Tkkkr_dot_left=[0,0,0,0,0,0]
d_right=[0,0,0,0,0,0]
d_left=[0,0,0,0,0,0]
Kp=np.identity(3)
Kd=Kd=np.identity(6)

PD_controller_right = rospy.Publisher("PD_controller_right", Accel, queue_size=10)
PD_controller_left  = rospy.Publisher("PD_controller_left", Accel, queue_size=10)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter_pos", Point, callback_pos)
    rospy.Subscriber("chatter_vel", Twist, callback_vel)
    rospy.Subscriber("chatter_acc", Accel, callback_acc)
    rospy.Subscriber("robot_states", WholeBodyState, callback)
    rospy.Subscriber("chatter_joint", String, callback_joint)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
        

def callback_joint(msg_joint):
    global joint
    joint=msg_joint.data

def callback_pos(msg_pos):
    global x_pos
    global y_pos
    global z_pos
    x_pos=msg_pos.x
    y_pos=msg_pos.y
    z_pos=msg_pos.z


def callback_vel(msg_vel):
    global x_vel
    global y_vel
    global z_vel
    x_vel=msg_vel.linear.x
    y_vel=msg_vel.linear.y
    z_vel=msg_vel.linear.z


def callback_acc(msg_acc):
    global x_acc
    global y_acc
    global z_acc
    x_acc=msg_acc.linear.x
    y_acc=msg_acc.linear.y
    z_acc=msg_acc.linear.z


def callback(msg):
    global Pkkr_right
    global Pkkr_left
    global Tkkkr_right
    global Tkkkr_left
    global Tkkkr_dot_right
    global Tkkkr_dot_left
    global d_right
    global d_left
    global Kp
    global Kd
    global publisher_PD_right

    
    p=dict()

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

    for contact in msg.contacts:
        name = contact.name
        # Contact pose
        pose = contact.pose
        vel=contact.velocity

        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = pinocchio.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                          pose.orientation.z)
        p[name] = pinocchio.SE3(quaternion, position)

        velocity = np.array([vel.linear.x, 
                           vel.linear.y, 
                           vel.linear.z, 
                           0, 0, 0])

        
        if((name=="right_sole_link") and (joint=="Right")):

            Pkkr_right=[pose.position.x-x_pos,
                        pose.position.y-y_pos,
                        pose.position.y-y_pos
                        ]

            Tkkkr_right=[vel.linear.x-x_vel,
                         vel.linear.y-y_vel,
                         vel.linear.z-z_vel,
                         0,0,0]

            Tkkkr_dot_right=[x_acc,
                             y_acc,
                             z_acc,
                             0,0,0]

            Tkkkr_left=[0,0,0,0,0,0]
            Tkkkr_dot_left=[0,0,0,0,0,0]



        if((name=="left_sole_link") and (joint=="Left")):
            
            Pkkr_left=[pose.position.x-x_pos,
                       pose.position.y-y_pos,
                       pose.position.z-z_pos
                       ]

            Tkkkr_left=[vel.linear.x-x_vel,
                vel.linear.y-y_vel,
                vel.linear.z-z_vel,
                0,0,0]

            Tkkkr_dot_left=[x_acc,
                            y_acc,
                            z_acc,
                            0,0,0]

            Tkkkr_right=[0,0,0,0,0,0]
            Tkkkr_dot_right=[0,0,0,0,0,0]


    print("Posicion error:")
    print(Pkkr_right)
    print(Pkkr_left)
    print("Twist error:")
    print(Tkkkr_right)
    print(Tkkkr_left)
    print("Aceleracion")
    print(Tkkkr_dot_right)
    print(Tkkkr_dot_left)

    Pkkr_right = np.array(Pkkr_right)
    Pkkr_left  = np.array(Pkkr_left)
    Pkkr_right = Kp.dot(Pkkr_right)
    Pkkr_left  = Kp.dot(Pkkr_left)

    Tkkkr_right = np.array(Tkkkr_right)
    Tkkkr_left  = np.array(Tkkkr_left)
    Tkkkr_right = Kd.dot(Tkkkr_right)
    Tkkkr_left  = Kd.dot(Tkkkr_left)

    Tkkkr_dot_right = np.array(Tkkkr_dot_right)
    Tkkkr_dot_left  = np.array(Tkkkr_dot_left)

    ############################################################################################################################################
    ######################################################## CONTROLADOR PD ####################################################################
    ############################################################################################################################################
   
    d_right[0] = Pkkr_right[0] + Tkkkr_right[0] + Tkkkr_dot_right[0]
    d_right[1] = Pkkr_right[1] + Tkkkr_right[1] + Tkkkr_dot_right[1]
    d_right[2] = Pkkr_right[2] + Tkkkr_right[2] + Tkkkr_dot_right[2]
    d_right[3] =       0       + Tkkkr_right[3] + Tkkkr_dot_right[3]
    d_right[4] =       0       + Tkkkr_right[4] + Tkkkr_dot_right[4]
    d_right[5] =       0       + Tkkkr_right[5] + Tkkkr_dot_right[5]

    d_left[0] = Pkkr_left[0] + Tkkkr_left[0] + Tkkkr_dot_left[0]
    d_left[1] = Pkkr_left[1] + Tkkkr_left[1] + Tkkkr_dot_left[1]
    d_left[2] = Pkkr_left[2] + Tkkkr_left[2] + Tkkkr_dot_left[2]
    d_left[3] =       0      + Tkkkr_left[3] + Tkkkr_dot_left[3]
    d_left[4] =       0      + Tkkkr_left[4] + Tkkkr_dot_left[4]
    d_left[5] =       0      + Tkkkr_left[5] + Tkkkr_dot_left[5]

    print("Referencias finales")
    print(d_right)
    print(d_left)
    print("--------------------------")

    publisher_PD_right = Accel()
    publisher_PD_right.linear.x  = d_right[0]
    publisher_PD_right.linear.y  = d_right[1]
    publisher_PD_right.linear.z  = d_right[2]
    publisher_PD_right.angular.x = d_right[3]
    publisher_PD_right.angular.y = d_right[4]
    publisher_PD_right.angular.z = d_right[5] 
    PD_controller_right.publish(publisher_PD_right)


    publisher_PD_left  = Accel()
    publisher_PD_left.linear.x  = d_left[0]
    publisher_PD_left.linear.y  = d_left[1]
    publisher_PD_left.linear.z  = d_left[2]
    publisher_PD_left.angular.x = d_left[3]
    publisher_PD_left.angular.y = d_left[4]
    publisher_PD_left.angular.z = d_left[5] 
    PD_controller_left.publish(publisher_PD_left)


if __name__ == '__main__':
    listener()

