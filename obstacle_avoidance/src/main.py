#! /usr/bin/env python
#-*- coding: utf-8 -*-

# ------------------------ import ------------------------
import rospy
from obstacle_detector.msg import Obstacles
from obstacle_avoidance.msg import avoid_angle

import math
import numpy as np

# ------------------- Global Variables -------------------
angle_range = None
gamma = None
d_max = None

w_robot = 0.20
theta_goal = 0.0

circles = None
min_angle = 0.0
origin = (0, 0)

# ------------------------ class -------------------------
class ObstacleAvoidance:
    def __init__(self) -> None:
        rospy.loginfo("Obstacle Avoidance is Created")

        # Param
        global angle_range
        global d_max
        global gamma
        
        angle_range = rospy.get_param("angle_range", 100)
        d_max = rospy.get_param("d_max", 2.0)
        gamma = rospy.get_param("gamma", 0.1)

        # Publisher
        self.pub = rospy.Publisher("/avoid_angle", avoid_angle, queue_size=10)

        # Subscriber
        rospy.Subscriber("/raw_obstacle", Obstacles, self.Obstacles_callback)
        
        # ros가 실행되는 동안 publish_data 함수 반복실행
        while not rospy.is_shutdown():
            self.publish_data()

    def Obstacles_callback(self, data):
        # Global Variables
        global circles
        global min_angle

        circles = data.circles

        min_angle = self.find_min_f_totoal()

    def publish_data(self):
        global min_angle

        # publish data 대입
        publishing_data = avoid_angle()
        publishing_data.angle = min_angle

        # publish
        self.pub.publish(publishing_data)

        rospy.loginfo("publish min angle : %f", min_angle)

    def find_min_f_totoal(self):
        f_total = []

        for angle in range(-1*angle_range, angle_range+1):
            f_total.append(self.calc_f_total(angle))

        return min(f_total)

    def calc_f_total(self, theta_i):
        f_rep = self.calc_f_rep(theta_i)
        f_att = self.calc_f_att(theta_i)
        f_total = f_rep + f_att

        return f_total

    def calc_f_rep(self, theta_i):
        global circles

        f_rep = 0

        for circle in circles:
            theta_k = self.calc_theta_k(circle.center)
            d_k = self.calc_d_k(circle.center)
            f_k = self.calc_f_k(theta_i, theta_k, d_k, circle.true_radius)
            f_rep += f_k

        return f_rep

    def calc_theta_k(self, point):
        global origin

        return self.calc_angle(origin, (point.x, point.y))

    def calc_d_k(self, point):
        global origin

        return self.calc_distance(origin, (point.x, point.y))

    def calc_f_k(self, theta_i, theta_k, d_k, radius):
        A_k = self.calc_A_k(d_k)
        sigma_k = self.calc_sigma_k(radius, d_k)

        return A_k * np.exp(-1*((theta_k-theta_i)**2)/(2*(sigma_k**2)))

    def calc_A_k(self, d_k):
        global d_max

        tilde_d_k = d_max - d_k
        A_k =tilde_d_k * np.exp(0.5)

        return A_k
    
    def calc_sigma_k(self, radius, d_k):
        global w_robot

        return math.atan2(radius+w_robot/2, d_k)

    def calc_f_att(self, theta_i):
        global gamma
        global theta_goal

        return gamma * abs(theta_goal - theta_i)

    def calc_distance(point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        # Calculate the differences in x and y coordinates
        dx = x2 - x1
        dy = y2 - y1

        # Calculate the distance using the Pythagorean theorem
        distance = math.sqrt(dx**2 + dy**2)

        return distance

    def calc_angle(point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        # Lidar X-axis transformation
        x1, x2 = -x1, -x2

        # Calculate the differences in x and y coordinates
        dx = x2 - x1
        dy = y2 - y1

        # Calculate the angle in radians using the arctan2 function
        angle_rad = math.atan2(dy, dx)

        # Convert the angle from radians to degrees
        angle_deg = math.degrees(angle_rad)

        return angle_deg

# ------------------------- run --------------------------
def run():
    rospy.init_node("obstacle_avoidance")
    new_class = ObstacleAvoidance()
    rospy.spin()

# ----------------------- __name__ ----------------------
if __name__=='__main__':
    run()