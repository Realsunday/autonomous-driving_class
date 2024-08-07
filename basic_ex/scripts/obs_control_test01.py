#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import math
import time

class OBS_Control:  
    def __init__(self):  
        # ROS 노드를 초기화합니다. 
        rospy.init_node("control_node")  # ROS 1단계(필수): 노드 이름 정의

        # 구독
        rospy.Subscriber("/dist90R", Float32, self.dist90R_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정
        rospy.Subscriber("/dist75R", Float32, self.dist75R_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정
        rospy.Subscriber("/dist90L", Float32, self.dist90L_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정
        rospy.Subscriber("/dist75L", Float32, self.dist75L_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정
        rospy.Subscriber("/obsR", Bool, self.obsR_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정
        rospy.Subscriber("/obsL", Bool, self.obsL_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

        # 발행
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  

        # 메시지 타입 설정 및 초기화
        self.cmd_msg = Twist()
        self.obsR = False
        self.obsL = False
        self.dist90R = 0.5
        self.dist90L = 0.5
        self.dist75R = 0.5
        self.dist75L = 0.5
        self.wall_dist = 0.2
        self.kp = 0.01
        self.pre_steer = 0

        # 상태 변수 초기화
        self.state = 'LEFT_WALL_FOLLOW'

        # ROS 퍼블리셔 주기 설정
        self.rate = rospy.Rate(2)

    def dist90R_CB(self, data):
        self.dist90R = data.data 
        # print('dist90R:',self.dist90R)
    def dist75R_CB(self, data):
        self.dist75R = data.data  
    def dist90L_CB(self, data):
        self.dist90L = data.data  
    def dist75L_CB(self, data):
        self.dist75L = data.data  
    def obsR_CB(self, data):
        self.obsR = data.data  
        # print('obsR:',self.obsR)
    def obsL_CB(self, data):        
        self.obsL = data.data  
        # print('obsL:',self.obsL)

    def control_right(self):
        speed = 0.2
        if self.dist90R == 0:
            speed = 0.0
            steer = self.pre_steer
            print('dist90R = 0')
        else:
            value = self.dist90R / self.dist75R
            if value > 1:
                value = 1
            elif value < -1:
                value = -1
            theta = math.acos(value) * 180 / math.pi
            steer = (15 - theta) * math.pi / 180 + self.kp * (self.wall_dist - self.dist90R)
            print('theta:', theta)
            print('steer:', steer)
        self.cmd_msg.linear.x = speed
        self.cmd_msg.angular.z = steer
        self.pub.publish(self.cmd_msg)
        self.pre_steer = steer
        print('Following the right wall')

    def control_left(self):
        speed = 0.2
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
            steer = (15 - theta) * math.pi / 180 + self.kp * (self.wall_dist - self.dist90L)
            print('theta:', theta)
            print('steer:', steer)
        self.cmd_msg.linear.x = speed
        self.cmd_msg.angular.z = -steer  # 왼쪽으로 회전하기 위해 조향각에 음수 부호를 추가
        self.pub.publish(self.cmd_msg)
        self.pre_steer = steer
        print('Following the left wall')

    def control_left_turn(self, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_msg.linear.x = 0.1
            self.cmd_msg.angular.z = 0.3
            self.pub.publish(self.cmd_msg)
            self.rate.sleep()
        self.state = 'RIGHT_WALL_FOLLOW'

    def control_stop(self):
        speed = 0
        steer = 0
        self.cmd_msg.linear.x = speed
        self.cmd_msg.angular.z = steer
        self.pub.publish(self.cmd_msg)
        print('Stopping')

    def control(self):
        if self.state == 'LEFT_WALL_FOLLOW':
            if self.obsL:
                self.control_left()
                print("!!!!!!!!!!!!!!!!!!!1")
            else:
                self.state = 'LEFT_TURN'
        elif self.state == 'LEFT_TURN':
            self.control_left_turn(3)  # 2초 동안 왼쪽으로 턴
            print("######################")
        elif self.state == 'RIGHT_WALL_FOLLOW':
            if self.obsR:
                self.control_right()
            else:
                self.control_stop() # 수정 필요함. 더 구체화적으로 구성.
        else:
            self.control_stop()


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    OBS_driving = OBS_Control()  # 클래스(Class_Name)를 변수(class_name)에 저장
    while not rospy.is_shutdown():
        OBS_driving.control()
        OBS_driving.rate.sleep()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()