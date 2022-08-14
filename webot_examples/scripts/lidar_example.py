#! /usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
# from sensor_msgs.msg import LaserScan
from obstacle_detector.msg import Obstacles
from webot_examples.msg import lidar_msg
import math

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        # self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        # rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("raw_obstacles", Obstacles, self.obstacles_callback)
        
    # def lidar_callback(self, _data):
    #     # 1단계
    #     theta_rad = []
    #     for i in range(len(_data.ranges)):
    #         tmp_theta = _data.angle_min + i * _data.angle_increment
    #         if tmp_theta >= _data.angle_max:
    #             tmp_theta = _data.angle_max
    #         theta_rad.append(tmp_theta)
        
    #     # 2단계
    #     min_idx = 0
    #     min_range = 1e9

    #     for i, distance in enumerate(_data.ranges):
    #         if distance < min_range:
    #             min_range = distance
    #             min_idx = i
        
    #     # 3단계
    #     x = min_range * math.cos(theta_rad[min_idx])
    #     y = min_range * math.sin(theta_rad[min_idx])

    #     #4단계
    #     min_deg = theta_rad[min_idx] * 180 / math.pi
    #     rospy.loginfo("\nMinimum Range (r, theta, x, y) = ({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(min_range, min_deg, x, y))

    def obstacles_callback(self, _data):
        circles = _data.circles

        if circles:
            self.state_ready(circles)

    def state_ready(self, circles):
        pass

def run():
    rospy.init_node("lidar_example")
    new_class = LidarReceiver()
    rospy.spin()

if __name__=='__main__':
    run()
