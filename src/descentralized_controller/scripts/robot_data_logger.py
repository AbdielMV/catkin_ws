#!/usr/bin/env python

import rospy
import csv
from whole_body_state_msgs.msg import WholeBodyState

class DataLogger:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('data_logger', anonymous=True)

        # Parameters
        self.output_file = rospy.get_param("~output_file", "robot_states.csv")
        self.topic_name = rospy.get_param("~topic_name", "/robot_states")

        # Open CSV file and write headers
        self.csv_file = open(self.output_file, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Name', 'Position', 'Velocity', 'Effort'])

        # Subscribe to the topic
        rospy.Subscriber(self.topic_name, WholeBodyState, self.callback)

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("DataLogger node initialized. Listening to %s and writing to %s", 
                      self.topic_name, self.output_file)

    def callback(self, data):
        # Extract joint data
        for joint in data.joints:
            name = joint.name
            position = joint.position
            velocity = joint.velocity
            effort = joint.effort
            time_now = data.time

            # Write to CSV
            self.csv_writer.writerow([time_now, name, position, velocity, effort])

    def shutdown(self):
        # Close the CSV file on shutdown
        if not self.csv_file.closed:
            self.csv_file.close()
            rospy.loginfo("DataLogger node shutting down. File saved: %s", self.output_file)

if __name__ == '__main__':
    try:
        logger = DataLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught")
