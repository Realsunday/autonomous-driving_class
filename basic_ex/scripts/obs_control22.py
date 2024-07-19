#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import math

class OBS_Control:
    def __init__(self):
        rospy.init_node("control_node")

        rospy.Subscriber("/dist90R", Float32, self.dist90R_CB)
        rospy.Subscriber("/dist75R", Float32, self.dist75R_CB)
        rospy.Subscriber("/dist90L", Float32, self.dist90L_CB)
        rospy.Subscriber("/dist75L", Float32, self.dist75L_CB)
        rospy.Subscriber("/obsR", Bool, self.obsR_CB)
        rospy.Subscriber("/obsL", Bool, self.obsL_CB)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.cmd_msg = Twist()
        self.obsR = False
        self.obsL = False
        self.dist90R = 0.5
        self.dist90L = 0.5
        self.dist75R = 0.5
        self.dist75L = 0.5
        self.wall_dist = 0.1
        self.kp = 0.01
        self.pre_steer = 0

        self.rate = rospy.Rate(2)

    def dist90R_CB(self, data):
        self.dist90R = data.data
        print('self.dist90R:', self.dist90R)

    def dist75R_CB(self, data):
        self.dist75R = data.data

    def dist90L_CB(self, data):
        self.dist90L = data.data
        print('self.dist90L:', self.dist90L)

    def dist75L_CB(self, data):
        self.dist75L = data.data

    def obsR_CB(self, data):
        self.obsR = data.data

    def obsL_CB(self, data):
        self.obsL = data.data

    def control(self):
        # 왼쪽과 오른쪽 거리의 평균으로 조향각을 계산
        avg_dist = (self.dist90R + self.dist90L) / 2.0
        value = avg_dist / self.wall_dist
        print('avg_dist;', avg_dist)
        
        if value > 1:
            value = 1
        elif value < -1:
            value = -1
        
        theta = math.acos(value) * 180 / math.pi
        if theta < 15:
            steer = 0
        else:
            steer = (15 - theta) * math.pi / 180 + self.kp * (self.wall_dist - avg_dist)
            print('theta:', theta)
            print('steer:', steer)
            print('벽')

        # 왼쪽 벽이나 오른쪽 벽 중 하나만 인식되는 경우 반대 방향으로 주행
        if self.obsR and not self.obsL:
            steer = 2
        elif self.obsL and not self.obsR:
            steer = -2

        self.cmd_msg.linear.x = 0.3
        self.cmd_msg.angular.z = steer
        self.pub.publish(self.cmd_msg)
        self.pre_steer = steer

def main():
    OBS_driving = OBS_Control()
    while not rospy.is_shutdown():
        OBS_driving.control()
        OBS_driving.rate.sleep()

if __name__ == "__main__":
    main()