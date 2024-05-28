#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class move_limo:
    def __init__(self):
        rospy.init_node('control')

        # 기본 설정 변수 초기화 
        self.BASE_ANGLE = 0
        self.BASE_SPEED = 0.85
        self.left_x = 0
        self.right_x = 0
        self.KP = 0.0084 # 각도 조정을 위한 비례 상수
        self.yellow_ANGLE = None  # 노란색 차선 조향 각도 초기화
        self.white_ANGLE = None  # 흰색 차선 조향 각도 초기화
        self.white_ratio = 0 
        self.obs_dist = 100 # 장애물까지의 초기 거리
        
        # 구독 설정
        rospy.Subscriber("/left_x", Int32, self.left_x_cb, queue_size=1)
        rospy.Subscriber("/right_x", Int32, self.right_x_cb, queue_size=1)
        rospy.Subscriber("/white_ratio", Int32, self.white_ratio_cb)
        rospy.Subscriber("obstacle_dist", Int32, self.obstacle_dist_fn)
        # 발행 설정 
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # 루프 속도 설정
        self.rate = rospy.Rate(20)



    def obstacle_dist_fn(self, data):
        # 장애물 거리 콜백 함수
        self.obs_dist = data.data
        #print("obstacle_dist:", self.obs_dist)

    def white_ratio_cb(self, data):
        # 흰색 비율 콜백 함수
        self.white_ratio = data.data
        #print("white_ratio:", self.white_ratio)
    
    def left_x_cb(self, data):
        # 왼쪽 차선 위치 콜백 함수
        #print("Updated left_x")
        #print("left_x callback called with data:", data.data)
        if data.data == 0:
             self.left_x = None
        else:
            self.left_x = data.data
            #print(f"Updated left_x: {self.left_x}")
        
    def right_x_cb(self, data):
        # 오른쪽 차선 위치 콜백 함
        #print("right_x callback called with data:", data.data)
        if data.data == 0:
            self.right_x = None
        else:
            self.right_x = data.data
            #print(f"Updated right_x: {self.right_x}")


    def drive_control(self):
          #주행 제어 로직
         try:
             drive = Twist()
             drive.linear.x = self.BASE_SPEED

             print(f"left_x: {self.left_x}, right_x: {self.right_x}")
             print("speed:", drive.linear.x)

             if self.left_x is not None and self.right_x is not None:
                  #두 차선 모두 감지된 경우
                 combined_angle = (self.KP * (130 - self.left_x) + self.KP * (230 - self.right_x) / 2)
                 drive.angular.z = combined_angle
                 # print("Combined steering angle:", combined_angle)

             elif self.right_x is not None:
                  #오른쪽 차선만 감지된 경우
                 self.white_ANGLE = self.KP * (230 - self.right_x)
                 drive.angular.z = self.white_ANGLE
                 print("white_ANGLE:", self.white_ANGLE)
                 print("********")
                 print("white:", drive.linear.x)

             elif self.left_x is not None:
                  #왼쪽 차선만 감지된 경우
                 #self.yellow_ANGLE = self.KP * (130 - self.left_x)
                 drive.angular.z = -1.85 #self.yellow_ANGLE
                 print("left****************")
                 # print("yellow_ANGLE:", self.yellow_ANGLE)

             else:
                 # No lane detected
                 drive.linear.x = self.BASE_SPEED  # Reduce linear velocity by 90%
                 drive.angular.z = -1.5  # Set a default angular velocity

                #print(f"Calculated yellow_ANGLE: {self.yellow_ANGLE}")
                #print(f"Calculated white_ANGLE: {self.white_ANGLE}")

              #흰색 비율이 특정 값 이상일때 조향 
             if self.white_ratio >= 23:
                 drive.linear.x = 2
                 drive.angular.z = -2
             else:
                 drive.linear.x = self.BASE_SPEED 

              #장애물 거리에 따른 속도 조정 
             if self.obs_dist < 25:
                 drive.linear.x = 0
                 drive.angular.z = 0
                 print("stop")
             elif self.obs_dist < 45:
                 drive.linear.x = max(drive.linear.x * 0.3, 0)   #장애물 근접 시 추가 속도 저하
                 print("speed down")
            
             self.drive_pub.publish(drive)
             self.rate.sleep()
         except Exception as e:
             print("error:", e)


if __name__ == '__main__':
    MoveCar = move_limo()
    try:
        while not rospy.is_shutdown():
            MoveCar.drive_control()
    except KeyboardInterrupt:
        print("program down")
