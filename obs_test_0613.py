#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import math

class Obs_detect:
    def __init__(self):
        rospy.init_node("obs_detect_node")

        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)

        self.pub_dist90_R = rospy.Publisher("/dist90R", Float32, queue_size=1)
        self.pub_dist75_R = rospy.Publisher("/dist75R", Float32, queue_size=1)
        self.pub_dist90_L = rospy.Publisher("/dist90L", Float32, queue_size=1)
        self.pub_dist75_L = rospy.Publisher("/dist75L", Float32, queue_size=1)
        self.pub_obs_R  = rospy.Publisher("/obsR", Bool, queue_size=1)
        self.pub_obs_L = rospy.Publisher("/obsL", Bool, queue_size=1)

        self.laser_msg = LaserScan()
        self.rate = rospy.Rate(5)
        self.laser_flag = False
        self.degrees = []
        self.degrees_flag = False

    def lidar_CB(self, msg):
        if msg != -1:
            self.laser_msg = msg
            self.laser_flag = True
        else:
            self.laser_flag = False

    def calculate_degrees(self):
        if self.degrees_flag == False:
            for i, v in enumerate(self.laser_msg.ranges):
                self.degrees.append((self.laser_msg.angle_min + self.laser_msg.angle_increment * i) * 180 / math.pi)
            self.degrees_flag = True

    def detect_obs(self):
        if self.degrees_flag == False:
            return [], [], [], [], [], []

        pub_dist90_R_list = []
        pub_dist75_R_list = []
        pub_dist90_L_list = []
        pub_dist75_L_list = []
        pub_obs_R_list = []
        pub_obs_L_list = []

        for i, n in enumerate(self.laser_msg.ranges):
            if n == float('inf'):
                continue

            # 각도 값
            angle = self.degrees[i]
            # x, y좌표로 변환 [라디안 단위]
            x = n * math.cos(angle * math.pi / 180)
            y = n * math.sin(angle * math.pi / 180) 

            # 장애물 거리 감지 (x, y 좌표에 대한)
            if 0.15 < y < 0.45 and 0 < x < 0.5:  # 왼쪽 장애물 감지
                pub_obs_L_list.append(n)
            if -0.45 < y < -0.15 and 0 < x < 0.5:  # 오른쪽 장애물 감지
                pub_obs_R_list.append(n)

            # 특정 각도 거리 감지
            if 0 < n < 0.5 and -91 < angle < -89:
                pub_dist90_R_list.append(n)
            if 0 < n < 0.5 and -76 < angle < -74:
                pub_dist75_R_list.append(n)
            if 0 < n < 0.5 and 89 < angle < 91:
                pub_dist90_L_list.append(n)
            if 0 < n < 0.5 and 74 < angle < 76:
                pub_dist75_L_list.append(n)

        return pub_dist90_R_list, pub_dist75_R_list, pub_dist90_L_list, pub_dist75_L_list, pub_obs_R_list, pub_obs_L_list

    def publish_distance_obs(self, dist90_R, dist75_R, dist90_L, dist75_L):
        # 특정 각도에 대한 장애물 감지 publish 
        # 오른쪽 90도, 75도에 대한 장애물 감지 
        if dist90_R:
            self.pub_dist90_R.publish(sum(dist90_R) / len(dist90_R))
        else:
            self.pub_dist90_R.publish(0.5)
        
        if dist75_R:
            self.pub_dist75_R.publish(sum(dist75_R) / len(dist75_R))
        else:
            self.pub_dist75_R.publish(0.5)
        # 왼쪽 90도, 75도에 대한 장애물 감지 
        if dist90_L:
            self.pub_dist90_L.publish(sum(dist90_L) / len(dist90_L))
        else:
            self.pub_dist90_L.publish(0.5)
        
        if dist75_L:
            self.pub_dist75_L.publish(sum(dist75_L) / len(dist75_L))
        else:
            self.pub_dist75_L.publish(0.5)

    def publish_right_obs(self, obs_R):
        if len(obs_R) > 5:
            self.pub_obs_R.publish(True)
            print("obs_R: ", True)
        else:
            self.pub_obs_R.publish(False)
            print("obs_R: ", False)

    def publish_left_obs(self, obs_L):
        if len(obs_L) > 5:
            self.pub_obs_L.publish(True)
            print("obs_L: ", True)
        else:
            self.pub_obs_L.publish(False)
            print("obs_L: ", False)
    

    def sense(self):
        if len(self.laser_msg.ranges) > 0:
            self.calculate_degrees()
            dist90_R, dist75_R, dist90_L, dist75_L, obs_R, obs_L = self.detect_obs()
            self.publish_distance_obs(dist90_R, dist75_R, dist90_L, dist75_L)
            self.publish_right_obs(obs_R)
            self.publish_left_obs(obs_L)

def main():
    obs_detect = Obs_detect()
    while not rospy.is_shutdown():
        obs_detect.sense()
        obs_detect.rate.sleep()

if __name__ == "__main__":
    main()
