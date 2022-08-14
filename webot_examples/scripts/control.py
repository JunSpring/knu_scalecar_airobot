#! /usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from lane_detection.msg import detected_msg

class Controller:
    def __init__(self):
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        
        self.rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_data()
            self.rate.sleep()

        self.waypoint_x = 0
        self.waypoint_y = 0

    def publish_data(self):
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now()
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = -0.2 # angle
        publishing_data.drive.speed = -2.0 

        self.drive_pub.publish(publishing_data)
        # rospy.loginfo("Publish Control Data!")

def run():
    rospy.init_node("control_example")
    Controller()
    rospy.spin()

if __name__ == "__main__":
    run()