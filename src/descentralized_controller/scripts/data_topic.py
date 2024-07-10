#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

def vector_publisher():
    rospy.init_node('vector_publisher_node', anonymous=True)
    pub = rospy.Publisher('/vector_data_topic', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        vector_data = np.random.rand(3)  # Generate a random 3D vector
        msg = Float32MultiArray(data=vector_data)
        pub.publish(msg)
        rospy.loginfo('Published: %s', vector_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        vector_publisher()
    except rospy.ROSInterruptException:
        pass
