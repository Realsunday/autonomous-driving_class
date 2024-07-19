#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from math import acos, pi

class OBS_Control:
    def __init__(self):
        rospy.init_node("control_node")

        rospy.Subscriber("/dist90R", Float32, self.dist90R_CB)
        rospy.Subscriber("/dist90L", Float32, self.dist90L_CB)
        rospy.Subscriber("/dist45R", Float32, self.dist45R_CB)
        rospy.Subscriber("/dist45L", Float32, self.dist45L_CB)
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
        self.dist45R = 0.5
        self.dist45L = 0.5
        self.wall_dist = 0.15  # 목표 거리
        self.kp = 10
        self.ki = 0.1
        self.kd = 1.0
        self.integral = 0
        self.prev_error = 0

        self.rate = rospy.Rate(10)

    def dist90R_CB(self, data):
        self.dist90R = data.data
    def dist90L_CB(self, data):
        self.dist90L = data.data
    def dist45R_CB(self, data):
        self.dist45R = data.data
    def dist45L_CB(self, data):
        self.dist45L = data.data
    def obsR_CB(self, data):
        self.obsR = data.data
    def obsL_CB(self, data):        
        self.obsL = data.data
    def obsC_CB(self, data):    
        self.obsC = data.data

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def control(self):  # 제어 함수
        if self.obsC and self.obsR and self.obsL: # Stop
            speed = 0
            steer = 0
        elif not self.obsC and self.obsR: # following the right wall
            speed = 0.2
            error = self.wall_dist - self.dist90R
            steer = self.pid_control(error)
        elif self.obsC and self.obsR: # turn left
            speed = 0.2
            steer = 20
        elif self.obsC and self.obsL: # turn right
            speed = 0.2
            steer = -20
        else:
            print('조건 없음')
            speed = 0
            steer = 0
        
        self.cmd_msg.linear.x = speed
        self.cmd_msg.angular.z = steer
        self.pub.publish(self.cmd_msg)

def main():
    OBS_driving = OBS_Control()
    while not rospy.is_shutdown():
        OBS_driving.control()
        OBS_driving.rate.sleep()

if __name__ == "__main__":
    main()