#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 이미지 데이터를 구독하고 이를 OpenCV 이미지로 변환하여 표시하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형 및 cv_bridge 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import CompressedImage
#from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from math import *
from time import *

# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class ImageProcessor:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("lane_detect_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/camera/rgb/image/raw/compressed" 토픽에서 CompressedImage 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.cam_CB)  # ROS 2단계(필수): 서브스크라이버 설정

        # CvBridge 모듈을 사용하여 이미지 메시지(CompressedImage)를 OpenCV 이미지로 변환할 준비를 합니다.
        self.bridge = CvBridge()
        self.img_msg = CompressedImage()  # 이미지 메시지를 저장할 변수를 초기화합니다.
        self.cam_flag = False  # 이미지 메시지 수신 여부를 확인하는 변수를 초기화합니다.
        # HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수들을 초기화합니다.
        self.L_H_Value = 0  # HSV 색상(Hue)의 하한(Hue, Saturation, Value)을 초기화합니다.
        self.L_S_Value = 0  # HSV 채도(Saturation)의 하한을 초기화합니다.
        self.L_V_Value = 0  # HSV 밝기(Value)의 하한을 초기화합니다.
        self.U_H_Value = 0  # HSV 색상의 상한을 초기화합니다.
        self.U_S_Value = 0  # HSV 채도의 상한을 초기화합니다.
        self.U_V_Value = 0  # HSV 밝기의 상한을 초기화합니다.
        self.L_R_Value = 0  # 좌측 또는 우측 뷰 선택을 나타내는 값을 초기화합니다.
        self.create_trackbar_flag = False  # 트랙바를 생성했는지 여부를 나타내는 플래그를 초기화합니다.
        self.original_window = "original_image"
        self.cropped_window = "croped_image"

    def cam_CB(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        # 이미지 메세지를 받아오기 위한 콜백 함수입니다.
        if msg != -1:
            self.img_msg = msg  # 유효한 메세지인 경우, 메세지를 self.img_msg로 저장합니다.
            self.cam_flag = True  # 이미지 메세지 수신 확인 변수를 True로 설정합니다.
        else:
            self.cam_flag = False  # 이미지 메세지 수신 확인 변수를 False로 설정합니다.
    
    def create_trackbar_init(self, cv_img):
        # HSV 범위를 조절하기 위한 트랙바를 생성하는 메서드입니다.
        # 이 메서드는 트랙바를 사용하여 이미지 처리 매개변수를 조정할 수 있도록 초기 설정을 수행합니다.

        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'original_image'로 설정합니다.
        cv2.namedWindow(self.cropped_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'croped_image'로 설정합니다.
        #cv2.namedWindow(self.control_window)  # OpenCV 창을 생성하고 창 이름을 'control'로 설정합니다.

        def hsv_track(value):
            # 트랙바 값이 변경될 때 호출되는 콜백 함수입니다. HSV 범위 트랙바의 현재 값을 업데이트합니다.
            self.L_H_Value = cv2.getTrackbarPos("Low_H", self.cropped_window)
            self.L_S_Value = cv2.getTrackbarPos("Low_S", self.cropped_window)
            self.L_V_Value = cv2.getTrackbarPos("Low_V", self.cropped_window)
            self.U_H_Value = cv2.getTrackbarPos("Up_H", self.cropped_window)
            self.U_S_Value = cv2.getTrackbarPos("Up_S", self.cropped_window)
            self.U_V_Value = cv2.getTrackbarPos("Up_V", self.cropped_window)
            #self.L_R_Value = cv2.getTrackbarPos("Left/Right view", self.cropped_window)

            # if self.L_R_Value != 1:
            #     # 왼쪽 뷰 선택 시 스티어링 표준 위치를 조절합니다.
            #     init_left_standard = self.init_standard // 2
            #     cv2.setTrackbarPos("Steering Standard", self.control_window, init_left_standard)
            # else:
            #     # 오른쪽 뷰 선택 시 스티어링 표준 위치를 조절합니다.
            #     init_right_standard = self.init_standard + self.init_standard // 2
            #     cv2.setTrackbarPos("Steering Standard", self.control_window, init_right_standard)

        # def control_track(value):
        #     # 트랙바 값이 변경될 때 호출되는 콜백 함수입니다. Stop/Go 및 Speed 트랙바 값 및 Steering_Standard를 업데이트합니다.
        #     self.Stop_or_Go = cv2.getTrackbarPos("Stop/Go", self.control_window)
        #     self.Speed_Value = cv2.getTrackbarPos("Speed", self.control_window)
        #     self.Steering_Standard = cv2.getTrackbarPos("Steering Standard", self.control_window)

        # 다양한 HSV 범위를 조정하기 위한 트랙바 생성
        cv2.createTrackbar("Low_H", self.cropped_window, 0, 255, hsv_track)  # H의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Low_S", self.cropped_window, 0, 255, hsv_track)  # S의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Low_V", self.cropped_window, 0, 255, hsv_track)  # V의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Up_H", self.cropped_window, 179, 255, hsv_track)  # H의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Up_S", self.cropped_window, 255, 255, hsv_track)  # S의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Up_V", self.cropped_window, 255, 255, hsv_track)  # V의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Left/Right view", self.cropped_window, 0, 1, hsv_track)  # 좌/우 뷰 선택 트랙바 생성
        #cv2.createTrackbar("Stop/Go", self.control_window, 0, 1, control_track)  # 정지/주행 트랙바 생성
        #cv2.createTrackbar("Speed", self.control_window, 0, 10, control_track)  # 속도 트랙바 생성
        #cv2.createTrackbar("Steering Standard", self.control_window, self.init_standard, cv_img.shape[1] // 2, control_track)  # 스티어링 표준 위치 트랙바 생성
        self.create_trackbar_flag = True  # 트랙바가 생성되었음을 표시합니다.
            
    def run(self):
        # 로봇의 메인 루프 함수로 이미지 처리 및 제어 동작이 수행됩니다.
        if self.cam_flag:
            img_msg = self.img_msg
            cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg)  # CvBridge 모듈을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변환합니다.
            if self.create_trackbar_flag == False:
                #self.init_standard = cv_img.shape[1] // 4  # 로봇 스티어링의 기준점을 설정합니다.
                self.create_trackbar_init(cv_img)  # 트랙바 초기화 함수를 호출합니다.
            cvt_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            # HSV 범위를 트랙바의 값으로 정의합니다.
            lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])  # HSV의 하한 범위를 정의합니다.
            upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])  # HSV의 상한 범위를 정의합니다.

            # HSV 범위를 사용하여 이미지에 마스크를 생성합니다.
            mask = cv2.inRange(cvt_hsv, lower, upper)

               # 원본 이미지에 마스크를 적용하여 차선만 있는 부분을 남긴 최종 결과 이미지를 생성합니다.
            hsv_img = cv2.bitwise_and(cv_img, cv_img, mask=mask)
            cv2.imshow("orig",cv_img)
            cv2.imshow("hsv",hsv_img)
            cv2.waitKey(1)
# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():
    try:
        image_processor = ImageProcessor()
        while not rospy.is_shutdown():
            # ImageProcessor 인스턴스를 생성하고 run 메서드를 호출하여 프로그램을 실행합니다.
            image_processor.run()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()  # 주요 프로그램이 시작됩니다.
    
    
    
    
  
