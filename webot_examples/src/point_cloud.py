#! /usr/bin/env python
#-*- coding: utf-8 -*-

# ------------------------ import ------------------------
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
from webot_examples.msg import lidar_msg

# ------------------------ 전역변수 ------------------------
# 좌우 ROI Parameter
parameter = 1
# 앞뒤 ROI Parameter
roi_parameter = 0

# ------------------------ class ------------------------
class rpScanfReceiver:
    def __init__(self):
        self.lscan = rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.Subscriber("/lidar_pub", lidar_msg, self.lidar_callback)
        self.pc_pub = rospy.Publisher("/pcl", PointCloud, queue_size=5)
 
    def callback(self, data):
        min_angle = data.angle_min
        min_range = data.range_min
        max_range = data.range_max
        angle_increment = data.angle_increment
        PC_data = PointCloud()
        PC_data.header = data.header

        for i, range in enumerate(data.ranges):
            x, y = calc_axis_xy(min_angle + angle_increment * i, range, min_range, max_range)
            if is_data(x, y):
                PC_data.points.append(Point32(x, y, 1))
        self.pc_pub.publish(PC_data)
    
    def lidar_callback(self, data):
        global parameter
        global roi_parameter

        if data.state == 3:
            parameter = 2
            roi_parameter = -0.25
        else:
            parameter = 1
            roi_parameter = 0

def calc_axis_xy(_theta, _distance, _min_range, _max_range):
    if _min_range <= _distance <= _max_range:
        x = np.cos(_theta) * _distance
        y = np.sin(_theta) * _distance
        return (x, y)   
    else:
        return (0, 0)

def is_data(_x, _y):
    global parameter
    global roi_parameter

    if [_x, _y] == [0, 0]:
        return False
    if  -2 <= _x <= 0 + roi_parameter and -0.75 * parameter <= _y <= 0.75:
        return True

    return False

# ------------------------ run ------------------------
def run():
    rospy.init_node('scan_py_receiver', anonymous=True)
    lc = rpScanfReceiver()
    rospy.spin()

# ------------------------ __name__ ------------------------
if __name__ == "__main__":
    run()