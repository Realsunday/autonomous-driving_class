#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import math

class OBS_Control:
    def __init__(self):
        rospy.init_node("control_node")

        rospy.Subscriber("/dist90R", Float32, self.dist90R_CB)
        rospy.Subscriber("/dist75R", Float32, self.dist75R_CB)
        rospy.Subscriber("/dist90L", Float32, self.dist90L_CB)
        rospy.Subscriber("/dist75L", Float32, self.dist75L_CB)
        rospy.Subscriber("/distObsC", Float32, self.distObsC_CB)
        rospy.Subscriber("/obsR", Bool, self.obsR_CB)
        rospy.Subscriber("/obsL", Bool, self.obsL_CB)
        rospy.Subscriber("/obsC", Bool, self.obsC_CB)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.cmd_msg = Twist()
        self.obsC = False
        self.obsR = False
        self.obsL = False
        self.dist90R = 0.5
        self.dist90L = 0.5
        self.dist75R = 0.5
        self.dist75L = 0.5
        self.distObsC = 0.5
        self.wall_dist = 0.4
        self.kp = 2.75
        self.pre_steer = 0

        self.rate = rospy.Rate(8)

    def dist90R_CB(self, data):
        self.dist90R = data.data
    def dist75R_CB(self, data):
        self.dist75R = data.data
    def dist90L_CB(self, data):
        self.dist90L = data.data
    def dist75L_CB(self, data):
        self.dist75L = data.data
    def distObsC_CB(self, data):
        self.distObsC = data.data
    def obsR_CB(self, data):
        self.obsR = data.data
    def obsL_CB(self, data):
        self.obsL = data.data
    def obsC_CB(self, data):
        self.obsC = data.data
    def control(self):
        # 기본 속도 및 조향 각도 초기화
        speed = 0.45
        steer = -20
        

        if not self.obsC and self.obsL and 0.25 < self.dist75L:  # 왼쪽 벽 따라가기 (멀리 있을 때)
            speed = 0.42
            if self.dist90L == 0:
                speed = 0.0
                steer = self.pre_steer
                print('dist90L = 0')
        
            else:
                value = self.dist90L / self.dist75L
                if value > 1:
                    value = 1
                elif value < -1:
                    value = -1

            theta = math.acos(value) * 180 / math.pi
            
            # theta의 최대값을 40으로 제한
            if theta > 23:
                theta = 10

            # print('theta:', theta)
            steer = ((15 - theta) * math.pi / 180 + self.kp * (self.wall_dist - self.dist90L))
            # print("steer:", steer)
            print('Follow LEFT wall', steer)

        elif not self.obsC and self.dist75L < 0.249:  # 왼쪽 벽 fllow(가까이)
            speed = 0.37
            steer = -0.6
            print("!!!!!!!!!!left warning!!!!!!!!!!")

        elif 0.15 < self.distObsC < 0.6 and self.obsL and 0.4 < self.dist75R and 0.4 < self.dist90R: #오른쪽으로 조향
                #if 0.31 < self.distObsC < 0.59:  
                speed = 0.35
                steer = -20
                print("jicjin")

                """elif self.distObsC < 0.3:
                    speed = 0.25
                    steer = -20
                    print("turn")"""

        elif not self.obsC and self.dist90R < 0.51 and self.dist75R < 0.5:  
            # 특이 케이스 
            speed = 0.4
            steer = 0.5
            print('Turning left1')

        
        elif self.obsC and self.dist75R < 0.5:  
            # 처음 왼쪽으로 하는 구간 
            speed = 0.25
            steer = 20
            print('Turning left2')

        elif self.obsC and not self.obsR and not self.obsL:  
            speed = 0.25
            steer = 20
            print('Turning left3')

        elif self.dist75R < 0.35 or self.dist90R < 0.35:
            speed = 0.25
            steer = 20
            print("right warning!!!!!!!!!!!")
        
        elif self.obsC < 0.2:
            speed = 0.25
            steer = 20

        else:
            print('No specific condition met')
        """
            elif self.obsC and not self.obsR and not self.obsL:  # 왼쪽으로 회전
            speed = 0.4
            steer = -20
            print('Turning right (center)')
            
            """

        self.cmd_msg.linear.x = speed
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