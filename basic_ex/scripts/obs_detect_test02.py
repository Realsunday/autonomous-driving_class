#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool
import math
import statistics

class LaserProcessor:
    def __init__(self):
        rospy.init_node("obs_detect_node")

        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)

        self.pub_dist90_R = rospy.Publisher("/dist90R", Int32, queue_size=1)
        self.pub_dist90_L = rospy.Publisher("/dist90L", Int32, queue_size=1)
        self.pub_dist45_R = rospy.Publisher("/dist45R", Int32, queue_size=1)
        self.pub_dist45_L = rospy.Publisher("/dist45L", Int32, queue_size=1)
        self.pub_obs_R = rospy.Publisher("/obsR", Bool, queue_size=1)
        self.pub_obs_L = rospy.Publisher("/obsL", Bool, queue_size=1)
        self.pub_obs_C = rospy.Publisher("/obsC", Bool, queue_size=1)

        self.laser_msg = None
        self.rate = rospy.Rate(10)
        self.degrees = []
        self.degrees_flag = False

    def lidar_CB(self, msg):
        self.laser_msg = msg

    def process_lidar(self):
        if self.laser_msg is None:
            return

        pub_dist90_R_list = []
        pub_dist90_L_list = []
        pub_dist45_R_list = []
        pub_dist45_L_list = []
        pub_obs_R_list = []
        pub_obs_L_list = []
        pub_obs_C_list = []

        if not self.degrees_flag:
            for i in range(len(self.laser_msg.ranges)):
                angle = (self.laser_msg.angle_min + self.laser_msg.angle_increment * i) * 180 / math.pi
                self.degrees.append(angle)
            self.degrees_flag = True

        for i, n in enumerate(self.laser_msg.ranges):
            x = n * math.cos(self.degrees[i] * math.pi / 180)
            y = n * math.sin(self.degrees[i] * math.pi / 180)

            if 0.15 < y < 0.45 and 0 < x < 0.5:
                pub_obs_R_list.append(n)
            if -0.15 < y < 0.15 and 0 < x < 0.5:
                pub_obs_C_list.append(n)
            if -0.45 < y < -0.15 and 0 < x < 0.5:
                pub_obs_L_list.append(n)

            if 0 < n < 0.5 and -91 < self.degrees[i] < -89:
                pub_dist90_R_list.append(n)
            if 0 < n < 0.5 and -46 < self.degrees[i] < -44:
                pub_dist45_R_list.append(n)
            if 0 < n < 0.5 and 89 < self.degrees[i] < 91:
                pub_dist90_L_list.append(n)
            if 0 < n < 0.5 and 44 < self.degrees[i] < 46:
                pub_dist45_L_list.append(n)

        self.publish_data(pub_dist90_R_list, self.pub_dist90_R)
        self.publish_data(pub_dist45_R_list, self.pub_dist45_R)
        self.publish_data(pub_dist90_L_list, self.pub_dist90_L)
        self.publish_data(pub_dist45_L_list, self.pub_dist45_L)

        self.publish_boolean(pub_obs_R_list, self.pub_obs_R)
        self.publish_boolean(pub_obs_C_list, self.pub_obs_C)
        self.publish_boolean(pub_obs_L_list, self.pub_obs_L)

    def publish_data(self, data_list, publisher):
        if data_list:
            publisher.publish(int(statistics.mean(data_list) * 100))
        else:
            publisher.publish(50)

    def publish_boolean(self, data_list, publisher):
        if len(data_list) > 3:
            publisher.publish(True)
        else:
            publisher.publish(False)

    def run(self):
        while not rospy.is_shutdown():
            self.process_lidar()
            self.rate.sleep()

if __name__ == "__main__":
    processor = LaserProcessor()
    processor.run()