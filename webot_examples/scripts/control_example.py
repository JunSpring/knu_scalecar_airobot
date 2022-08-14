#! /usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from lane_detection.msg import detected_msg

class Controller:
    def __init__(self):
        rospy.Subscriber("/lane_pub", detected_msg, self.lane_callback)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 5), self.timer_callback)
        # self.rate = rospy.Rate(5)
        # while not rospy.is_shutdown():
        #     self.publish_data()
        #     self.rate.sleep()

        self.waypoint_x = 0
        self.waypoint_y = 0

    def lane_callback(self, _data):
        self.waypoint_x = _data.xdetected
        self.waypoint_y = _data.ydetected

        str = f"x : {self.waypoint_x}\ty : {self.waypoint_y}"
        # rospy.loginfo(str)
        
    def timer_callback(self, _event):
        if self.waypoint_x == 0 and self.waypoint_y == 0:
            self.stop()
            return

        self.follow_lane()

    def calc_angle_speed(self):
        x = self.waypoint_x - 160
        y = 240 - self.waypoint_y
        parameter = 1

        try:
            angle = math.atan(x / y) / math.pi * 2 * 0.34 / parameter
        except:
            angle = 0
        speed = 2.5 - abs(angle) / 0.34 / parameter * 2.5

        str = f"x : {x}\ty : {y}"
        # rospy.loginfo(str)

        return angle, speed

    def follow_lane(self):
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now()
        publishing_data.header.frame_id = "base_link"

        angle, speed = self.calc_angle_speed()
        publishing_data.drive.steering_angle = -0.2 # angle
        publishing_data.drive.speed = -2.0 # speed
        str = f"speed : {speed}\tangle : {angle}"
        rospy.loginfo(str)

        self.drive_pub.publish(publishing_data)
        # rospy.loginfo("Publish Control Data!")

    def stop(self):
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now()
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = 0.0
        self.drive_pub.publish(publishing_data)
        rospy.loginfo("Stop Vehicle")

def run():
    rospy.init_node("control_example")
    Controller()
    rospy.spin()

if __name__ == "__main__":
    run()