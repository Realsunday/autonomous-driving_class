#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg,geometry_msgs.msg) 및 수학 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import math
import statistics

# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 클래스 이름 정의
    def __init__(self):  # 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("obs_detect_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/scan" 토픽에서 LaserScan 메시지를 구독하고, 콜백 함수(lidar_CB)를 호출합니다.
        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)  # 서브스크라이버 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        self.pub_dist90_R = rospy.Publisher("/dist90R", Int32, queue_size=1)  # 퍼블리셔 설정
        self.pub_dist90_L = rospy.Publisher("/dist90L", Int32, queue_size=1)  # 퍼블리셔 설정
        self.pub_dist45_R = rospy.Publisher("/dist45R", Int32, queue_size=1)  # 퍼블리셔 설정
        self.pub_dist45_L = rospy.Publisher("/dist45L", Int32, queue_size=1)  # 퍼블리셔 설정
        self.pub_obs_R  = rospy.Publisher("/obsR", Bool, queue_size=1)  # 퍼블리셔 설정
        self.pub_obs_L = rospy.Publisher("/obsL", Bool, queue_size=1)  # 퍼블리셔 설정
        self.pub_obs_C = rospy.Publisher("/obsC", Bool, queue_size=1)  # 퍼블리셔 설정

        self.laser_msg = LaserScan()  # LaserScan 메시지 타입 설정 및 초기화
        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정
        self.laser_flag = False  # LaserScan 메시지 수신 확인을 위한 변수 설정
        self.degrees = []  # 각도를 저장할 리스트 설정
        self.degrees_flag = False  # 각도 리스트 저장 확인을 위한 변수 설정

    def lidar_CB(self, msg):
        if msg != -1:  # 유효한 메시지가 들어왔을 경우:
            self.laser_msg = msg  # LaserScan 메시지를 self.laser_msg에 저장
            self.laser_flag = True  # LaserScan 메시지 수신 확인 변수를 True로 저장
        else:
            self.laser_flag = False  # 유효하지 않은 메시지일 경우 LaserScan 메시지 수신 확인 변수를 False로 저장

    def sense(self):  # 감지 함수
        pub_dist90_R_lsit = []
        pub_dist90_L_lsit = []
        pub_dist45_R_lsit = []
        pub_dist45_L_lsit = []
        pub_obs_R_list = []
        pub_obs_L_list = []
        pub_obs_C_list = []
        
        if self.laser_flag == True:  # LaserScan 메시지 수신 확인 변수가 True일 경우:
            if self.degrees_flag == False:  # 각도 리스트 저장 확인 변수가 False인 경우:
                # LaserScan 메시지의 각도 정보를 계산하여 degrees 리스트에 저장
                for i, v in enumerate(self.laser_msg.ranges):
                    self.degrees.append((self.laser_msg.angle_min + self.laser_msg.angle_increment * i) * 180 / math.pi)
                self.degrees_flag = True
            for i, n in enumerate(self.ranges):
                ## x, y 좌표로 변환
                x = n * math.cos(self.degrees[i] * math.pi / 180)
                y = n * math.sin(self.degrees[i] * math.pi / 180)              
                # right obstacle detection
                if 0.15 < y < 0.45 and 0 < x < 0.5:
                    pub_obs_R_list.append(n)
                # front obstacle detection
                if -0.15 < y < 0.15 and 0 < x < 0.5:
                    pub_obs_C_list.append(n)
                # left obstacle detection
                if -0.45 < y < -0.15 and 0 < x < 0.5:
                    pub_obs_L_list.append(n)
                
                # 오른쪽 90도 거리 저장
                if 0 < n < 0.5 and -91 < self.degrees[i] < -89:
                    pub_dist90_R_lsit.append(n)
                # 오른쪽 45도 거리 저장
                if 0 < n < 0.5 and -46 < self.degrees[i] < -44:
                    pub_dist45_R_lsit.append(n)
                
                # 왼쪽 90도 거리 저장
                if 0 < n < 0.5 and 89 < self.degrees[i] < 91:
                    pub_dist90_L_lsit.append(n)
                # # 왼쪽 45도 거리 저장
                if 0 < n < 0.5 and 44 < self.degrees[i] < 46:
                    pub_dist45_L_lsit.append(n)
                
                                    
            if len(pub_dist90_R_lsit) > 0:
                self.pub_dist90_R.publish(statistics.mean(pub_dist90_R_lsit))
                # print("dist90_R: ", statistics.mean(pub_dist90_R_lsit))
            else:
                self.pub_dist90_R.publish(0.2)
                # print("dist90_R: ", 0.5)
            if len(pub_dist45_R_lsit) > 0:
                self.pub_dist45_R.publish(statistics.mean(pub_dist45_R_lsit))
                # print("dist45_R: ", statistics.mean(pub_dist45_R_lsit))
            else:
                self.pub_dist45_R.publish(0.2)
                # print("dist45_R: ", 0.5)
            
            if len(pub_dist90_L_lsit) > 0:
                self.pub_dist90_L.publish(statistics.mean(pub_dist90_L_lsit))
                #print("dist90_L: ", statistics.mean(pub_dist90_L_lsit))
            else:
                self.pub_dist90_L.publish(0.2)
                #print("dist90_L: ", 0.5)
            if len(pub_dist45_L_lsit) > 0:
                self.pub_dist45_L.publish(statistics.mean(pub_dist45_L_lsit))
                #print("dist45_L: ", statistics.mean(pub_dist45_L_lsit))
            else:
                self.pub_dist45_L.publish(0.2)
                #print("dist45_L: ", 0.5)
            
            
            if len(pub_obs_R_list) > 3:
                self.pub_obs_R.publish(True)
                print("obs_R: ", True)
            else:
                self.pub_obs_R.publish(False)
                print("obs_R: ", False)
            if len(pub_obs_C_list) > 3:
                self.pub_obs_C.publish(True)
                print("obs_C: ", True)
            else:
                self.pub_obs_C.publish(False)
                print("obs_C: ", False)
            
            if len(pub_obs_L_list) > 3:
                self.pub_obs_L.publish(True)
                print("obs_L: ", True)
            else:
                self.pub_obs_L.publish(False)
                print("obs_L: ", False)
            
    '''               
    def think(self, degrees):  # 판단 함수
        object_degrees = []  # 장애물을 감지한 각도를 저장할 리스트
        for index, value in enumerate(self.laser_msg.ranges):  # LaserScan 메시지의 각도와 거리 정보를 반복하여 처리
            if 0 < self.laser_msg.ranges[index] < 0.30 and abs(degrees[index]) < 30:
                # 거리가 0에서 0.30 사이이고 각도가 30도 이하인 경우:
                object_degrees.append(degrees[index])  # object_degrees 리스트에 해당 각도 추가
            else:
                pass  # 그 외의 경우는 무시
        if len(object_degrees) > 5:  # 감지된 장애물의 각도 수가 5개 이상이면:
            motion = "STOP"  # 로봇을 정지 상태로 제어하기 위해 "STOP" 설정
        else:
            motion = "GO"  # 그렇지 않으면 로봇을 이동 상태로 제어하기 위해 "GO" 설정
        return motion  # motion 값 반환

    def act(self, motion):  # 동작 함수
        if motion == "STOP":  # motion 값이 "STOP"인 경우:
            speed = 0  # 로봇의 속도(speed)를 0으로 설정하여 정지
        else:
            speed = 1  # 그렇지 않으면 로봇의 속도(speed)를 1로 설정하여 이동
        self.ctrl_msg.linear.x = speed  # Twist 메시지의 선속도(linear.x)를 설정
        self.pub.publish(self.ctrl_msg)  # ROS 3단계(필수): 퍼블리셔 - 퍼블리시 실행

    def run(self):  # 전체 실행 함수
        if self.laser_flag == True:  # LaserScan 메시지 수신 확인 변수가 True인 경우:
            degrees = self.sense()  # sense 함수를 사용하여 각도 정보를 얻음
            motion = self.think(degrees)  # think 함수를 사용하여 판단
            self.act(motion)  # act 함수를 사용하여 로봇을 제어
        else:
            pass  # LaserScan 메시지 수신 확인이 False인 경우 패스
    '''

# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스 생성
    while not rospy.is_shutdown():
        class_name.run()


if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()