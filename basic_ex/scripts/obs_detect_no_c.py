#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import math

class Obs_detect:
    def __init__(self):
        rospy.init_node("obs_detect_node")

        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)

        self.pub_dist90_R = rospy.Publisher("/dist90R", Float32, queue_size=1)
        self.pub_dist75_R = rospy.Publisher("/dist75R", Float32, queue_size=1)
        self.pub_dist90_L = rospy.Publisher("/dist90L", Float32, queue_size=1)
        self.pub_dist75_L = rospy.Publisher("/dist75L", Float32, queue_size=1)
        self.pub_obs_R = rospy.Publisher("/obsR", Bool, queue_size=1)
        self.pub_obs_L = rospy.Publisher("/obsL", Bool, queue_size=1)

        self.laser_msg = LaserScan()
        self.rate = rospy.Rate(5)
        self.laser_flag = False
        self.degrees = []
        self.degrees_flag = False

    def lidar_CB(self, msg):
        if msg:
            self.laser_msg = msg
            self.laser_flag = True
        else:
            self.laser_flag = False

    def sense(self):
        if not self.laser_flag:
            return

        current_laser = self.laser_msg

        pub_dist90_R_list = []
        pub_dist90_L_list = []
        pub_dist75_R_list = []
        pub_dist75_L_list = []
        pub_obs_R_list = []
        pub_obs_L_list = []

        if len(current_laser.ranges) > 0:
            if not self.degrees_flag:
                for i, v in enumerate(current_laser.ranges):
                    self.degrees.append((current_laser.angle_min + current_laser.angle_increment * i) * 180 / math.pi)
                self.degrees_flag = True
                rospy.loginfo("각도 계산 완료")

            if self.degrees_flag:
                for i, n in enumerate(current_laser.ranges):
                    x = n * math.cos(self.degrees[i] * math.pi / 180)
                    y = n * math.sin(self.degrees[i] * math.pi / 180)
                    
                    if 0.03 < y < 0.6 and 0 < x < 0.4:
                        pub_obs_L_list.append(n)
                    if -0.6 < y < 0.02 and 0 < x < 0.4:
                        pub_obs_R_list.append(n)
                    if 0 < n < 0.8 and -5 < self.degrees[i] < 0:
                        pub_dist90_R_list.append(n)
                    if 0 < n < 0.5 and -76.5 < self.degrees[i] < -74.5:
                        pub_dist75_R_list.append(n)
                    if 0 < n < 1 and 89.5 < self.degrees[i] < 91.5:
                        pub_dist90_L_list.append(n)
                    if 0 < n < 1 and 67.5 < self.degrees[i] < 70.5:
                        pub_dist75_L_list.append(n)

                if pub_dist90_R_list:
                    dist90R = sum(pub_dist90_R_list) / len(pub_dist90_R_list)
                else:
                    dist90R = 0.8
                self.pub_dist90_R.publish(dist90R)
                print("/dist90R:", dist90R)

                if pub_dist75_R_list:
                    dist75R = sum(pub_dist75_R_list) / len(pub_dist75_R_list)
                else:
                    dist75R = 0.75
                self.pub_dist75_R.publish(dist75R)
                print("/dist75R:", dist75R)

                if pub_dist90_L_list:
                    dist90L = sum(pub_dist90_L_list) / len(pub_dist90_L_list)
                else:
                    dist90L = 1
                self.pub_dist90_L.publish(dist90L)
                print("/dist90L:", dist90L)

                if pub_dist75_L_list:
                    dist75L = sum(pub_dist75_L_list) / len(pub_dist75_L_list)
                else:
                    dist75L = 1
                self.pub_dist75_L.publish(dist75L)
                print("/dist75L:", dist75L)

                if len(pub_obs_R_list) > 5:
                    self.pub_obs_R.publish(True)
                else:
                    self.pub_obs_R.publish(False)

                if len(pub_obs_L_list) > 5:
                    self.pub_obs_L.publish(True)
                else:
                    self.pub_obs_L.publish(False)

def main():
    obs_detect = Obs_detect()
    while not rospy.is_shutdown():
        obs_detect.sense()
        obs_detect.rate.sleep()

if __name__ == "__main__":
    main()
