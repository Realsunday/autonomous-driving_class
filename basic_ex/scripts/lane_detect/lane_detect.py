#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 이미지 데이터를 구독하고 이를 OpenCV 이미지로 변환하여 표시하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형 및 cv_bridge 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import *
import cv2


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
        self.img_msg = CompressedImage()
        self.cam_flag = False  # 이미지 메시지 수신 여부를 확인하는 변수를 초기화합니다.
         # HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수들을 초기화합니다.
        self.L_H_Value = 0  # HSV 색상(Hue)의 하한(Hue, Saturation, Value)을 초기화합니다.
        self.L_S_Value = 0  # HSV 채도(Saturation)의 하한을 초기화합니다.
        self.L_V_Value = 0  # HSV 밝기(Value)의 하한을 초기화합니다.
        self.U_H_Value = 0  # HSV 색상의 상한을 초기화합니다.
        self.U_S_Value = 0  # HSV 채도의 상한을 초기화합니다.
        self.U_V_Value = 0  # HSV 밝기의 상한을 초기화합니다.
        self.L_R_Value = 0  # 좌측 또는 우측 뷰 선택을 나타내는 값을 초기화합니다.

    def cam_CB(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        # 이미지 메세지를 받아오기 위한 콜백 함수입니다.
        if msg != -1:
            self.img_msg = msg  # 유효한 메세지인 경우, 메세지를 self.img_msg로 저장합니다.
            self.cam_flag = True  # 이미지 메세지 수신 확인 변수를 True로 설정합니다.
        else:
            self.cam_flag = False  # 이미지 메세지 수신 확인 변수를 False로 설정합니다.



    def run(self):
        # 로봇의 메인 루프 함수로 이미지 처리 및 제어 동작이 수행됩니다.
        if self.cam_flag:
             img_msg = self.img_msg  # 로봇으로부터 수신한 이미지 메세지를 변수에 저장합니다.
             cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg)  # CvBridge 모듈을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변환합니다.
             if self.create_trackbar_flag == False:
            self.init_standard = cv_img.shape[1] // 4  # 로봇 스티어링의 기준점을 설정합니다.
            self.create_trackbar_init(cv_img)  # 트랙바 초기화 함수를 호출합니다.


             cv2.imshow("orig",cv_img)
             cv2.waitKey(1)
             

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
