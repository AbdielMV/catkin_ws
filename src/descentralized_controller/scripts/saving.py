#!/usr/bin/env python

import rospy
import numpy as np
import csv
from std_msgs.msg import Float32MultiArray

class DataSaver:
    def __init__(self):
        self.data = []
        rospy.init_node('data_saver_node', anonymous=True)
        rospy.Subscriber('/vector_data_topic', Float32MultiArray, self.callback)
        rospy.on_shutdown(self.save_to_csv)

    def callback(self, msg):
        self.data.append(msg.data)
        rospy.loginfo('Data received: %s', msg.data)

    def save_to_csv(self):
        with open('vector_data.csv', 'wb') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['X', 'Y', 'Z'])  # Assuming 3D vectors
            writer.writerows(self.data)
        rospy.loginfo('Data saved to vector_data.csv')

if __name__ == '__main__':
    try:
        data_saver = DataSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
