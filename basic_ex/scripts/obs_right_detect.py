#!/usr/bin/env python
# -*- coding: utf-8 -*-

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg,geometry_msgs.msg) 및 수학 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import math

# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Obs_detect:  # 클래스 이름 정의
    def __init__(self):  # 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("obs_detect_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/scan" 토픽에서 LaserScan 메시지를 구독하고, 콜백 함수(lidar_CB)를 호출합니다.
        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)  # 서브스크라이버 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        self.pub_dist90_R = rospy.Publisher("/dist90R", Float32, queue_size=1)  # 퍼블리셔 설정
        self.pub_dist75_R = rospy.Publisher("/dist75R", Float32, queue_size=1)  # 퍼블리셔 설정
        self.pub_dist90_L = rospy.Publisher("/dist90L", Float32, queue_size=1)  # 퍼블리셔 설정
        self.pub_dist75_L = rospy.Publisher("/dist75L", Float32, queue_size=1)  # 퍼블리셔 설정
        self.pub_obs_R  = rospy.Publisher("/obsR", Bool, queue_size=1)  # 퍼블리셔 설정
        self.pub_obs_L = rospy.Publisher("/obsL", Bool, queue_size=1)  # 퍼블리셔 설정
        self.pub_obs_C = rospy.Publisher("/obsC", Bool, queue_size=1)  # 퍼블리셔 설정

        self.laser_msg = LaserScan()  # LaserScan 메시지 타입 설정 및 초기화
        self.rate = rospy.Rate(5)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정
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
        
        curren_laser = self.laser_msg
        
        pub_dist90_R_list = []
        pub_dist90_L_list = []
        pub_dist75_R_list = []
        pub_dist75_L_list = []
        pub_obs_R_list = []
        pub_obs_L_list = []
        pub_obs_C_list = []
        
        if len(curren_laser.ranges) > 0:  # LaserScan 메시지 수신 확인 변수가 True일 경우:
            if self.degrees_flag == False:  # 각도 리스트 저장 확인 변수가 False인 경우:
                # LaserScan 메시지의 각도 정보를 계산하여 degrees 리스트에 저장
                print(curren_laser.ranges)
                for i, v in enumerate(curren_laser.ranges):
                    self.degrees.append((curren_laser.angle_min + curren_laser.angle_increment * i) * 180 / math.pi)
                self.degrees_flag = True
                print(self.degrees)
            if self.degrees_flag == True:
                for i, n in enumerate(curren_laser.ranges):
                    #if 5 < self.degrees[i] < 30:
                    #   print('test',n)

                    ## x, y 좌표로 변환
                    x = n * math.cos(self.degrees[i] * math.pi / 180)
                    y = n * math.sin(self.degrees[i] * math.pi / 180)              
                    # right obstacle detection
                    if 0 < y < 0.25 and 0 < x < 0.25:
                        pub_obs_L_list.append(n)
                    # front obstacle detectionself.ranges
                    if -0.1 < y < 0.1 and 0 < x < 0.7:
                        pub_obs_C_list.append(n)

                    # left obstacle detection
                    if -0.4 < y < -0.1 and 0 < x < 0.4:
                        pub_obs_R_list.append(n)
                    
                    # 오른쪽 90도 거리 저장
                    if 0 < n < 0.5 and -91 < self.degrees[i] < -89:
                        pub_dist90_R_list.append(n)
                    # 오른쪽 75도 거리 저장
                    if 0 < n < 0.5 and -76 < self.degrees[i] < -74:
                        pub_dist75_R_list.append(n)
                    
                    # # 왼쪽 90도 거리 저장
                    if 0 < n < 0.5 and 89 < self.degrees[i] < 91:
                        pub_dist90_L_list.append(n)
                    # 왼쪽 75도 거리 저장
                    if 0 < n < 0.5 and 74 < self.degrees[i] < 76:
                        pub_dist75_L_list.append(n)
                                            
                if len(pub_dist90_R_list) > 0:
                    self.pub_dist90_R.publish((sum(pub_dist90_R_list)/len(pub_dist90_R_list)))
                    print("dist90_R: ", sum(pub_dist90_R_list)/len(pub_dist90_R_list))
                else:
                    self.pub_dist90_R.publish(0.5)
                    print("dist90_R: ", 0.5)

                if len(pub_dist75_R_list) > 0:
                    self.pub_dist75_R.publish(sum(pub_dist75_R_list)/len(pub_dist75_R_list))
                    print("dist75_R: ", sum(pub_dist75_R_list)/len(pub_dist75_R_list))
                else:
                    self.pub_dist75_R.publish(0.5)
                    print("dist75_R: ", 0.5)
                
                if len(pub_dist90_L_list) > 0:
                    self.pub_dist90_L.publish(sum(pub_dist90_L_list)/len(pub_dist90_L_list))
                    print("dist90_L: ", sum(pub_dist90_L_list)/len(pub_dist90_L_list))
                else:
                    self.pub_dist90_L.publish(0.5)
                    print("dist90_L: ", 0.5)
                if len(pub_dist75_L_list) > 0:
                    self.pub_dist75_L.publish(sum(pub_dist75_L_list)/len(pub_dist75_L_list))
                    print("dist75_L: ", sum(pub_dist75_L_list)/len(pub_dist75_L_list))
                else:
                    self.pub_dist75_L.publish(0.5)
                    print("dist75_L: ", 0.5)

                if len(pub_obs_R_list) > 3:
                    self.pub_obs_R.publish(True)
                    print("obs_R: ", True)
                else:
                    self.pub_obs_R.publish(False)
                    print("obs_R: ", False)

                if len(pub_obs_C_list) > 1:
                    self.pub_obs_C.publish(True)
                    print("obs_C: ", True)
                else:
                    self.pub_obs_C.publish(False)
                    print("obs_C: ", False)

                if len(pub_obs_L_list) > 10000:
                    self.pub_obs_L.publish(True)
                    print("obs_L: ", True)
                else:
                    self.pub_obs_L.publish(False)
                    print("obs_L: ", False)
                

# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    obs_detect = Obs_detect()  # Class_Name 클래스의 인스턴스 생성
    while not rospy.is_shutdown():
        obs_detect.sense()
        obs_detect.rate.sleep()


if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()