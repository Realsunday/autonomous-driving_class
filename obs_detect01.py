#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class Lane_detect:  
    def __init__(self):  
        rospy.init_node("homework_node")
        # 구독
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.image_fn)  
        self.bridge = CvBridge() 
        self.img_msg = CompressedImage()  # 이미지 메시지 저장할 변수 초기화 
        self.cam_flag = False

        # 트랙바 초기값 설정 
        self.GaussianBlur_Value = 1 #가우시안 블러 커널 사이즈 (홀수)
        self.CannyLow_Value = 0 # 캐니 엣지 검출 최소 임곗값
        self.CannyHigh_Value = 0 # 캐니 엣지 검출 최대 임곗값

        # 첫 번째 HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수 초기화
        self.L_H_Value1 = 0
        self.L_S_Value1 = 0
        self.L_V_Value1 = 0
        self.U_H_Value1 = 0
        self.U_S_Value1 = 0
        self.U_V_Value1 = 0

        # 두 번째 첫 번째 HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수 초기화
        self.L_H_Value2 = 0
        self.L_S_Value2 = 0
        self.L_V_Value2 = 0
        self.U_H_Value2 = 0
        self.U_S_Value2 = 0
        self.U_V_Value2 = 0

        self.rate = rospy.Rate(20) # lane detect 주기 설정 

        self.create_trackbar_init()  # 트랙바 초기화
    
    def image_fn(self, msg): 
        if msg != -1:
            self.img_msg = msg
            self.cam_flag = True
        else:
            self.cam_flag = False

    def create_trackbar_init(self):
        self.hsv_window = 'HSV Settings'
        self.preprocess_window = 'Preprocess Settings'

        cv2.namedWindow(self.hsv_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.preprocess_window, cv2.WINDOW_NORMAL)

        def hsv_track(value):
            # White color trackbar
            self.L_H_Value1 = cv2.getTrackbarPos("Low_H1", self.hsv_window)
            self.L_S_Value1 = cv2.getTrackbarPos("Low_S1", self.hsv_window)
            self.L_V_Value1 = cv2.getTrackbarPos("Low_V1", self.hsv_window)
            self.U_H_Value1 = cv2.getTrackbarPos("Up_H1", self.hsv_window)
            self.U_S_Value1 = cv2.getTrackbarPos("Up_S1", self.hsv_window)
            self.U_V_Value1 = cv2.getTrackbarPos("Up_V1", self.hsv_window)

            # Yellow color trackbar
            self.L_H_Value2 = cv2.getTrackbarPos("Low_H2", self.hsv_window)
            self.L_S_Value2 = cv2.getTrackbarPos("Low_S2", self.hsv_window)
            self.L_V_Value2 = cv2.getTrackbarPos("Low_V2", self.hsv_window)
            self.U_H_Value2 = cv2.getTrackbarPos("Up_H2", self.hsv_window)
            self.U_S_Value2 = cv2.getTrackbarPos("Up_S2", self.hsv_window)
            self.U_V_Value2 = cv2.getTrackbarPos("Up_V2", self.hsv_window)

        def blur_track(value):
            self.GaussianBlur_Value = cv2.getTrackbarPos("GaussianBlur", self.preprocess_window)
            if self.GaussianBlur_Value % 2 == 0:
                self.GaussianBlur_Value += 1
            cv2.setTrackbarPos("GaussianBlur", self.preprocess_window, self.GaussianBlur_Value)

        def canny_track(value):
            self.CannyLow_Value = cv2.getTrackbarPos("CannyLow", self.preprocess_window)
            self.CannyHigh_Value = cv2.getTrackbarPos("CannyHigh", self.preprocess_window)

        # White color trackbars
        cv2.createTrackbar("Low_H1", self.hsv_window, 75, 255, hsv_track)
        cv2.createTrackbar("Low_S1", self.hsv_window, 0, 255, hsv_track)
        cv2.createTrackbar("Low_V1", self.hsv_window, 118, 255, hsv_track)
        cv2.createTrackbar("Up_H1", self.hsv_window, 255, 255, hsv_track)
        cv2.createTrackbar("Up_S1", self.hsv_window, 255, 255, hsv_track)
        cv2.createTrackbar("Up_V1", self.hsv_window, 255, 255, hsv_track)

        # Yellow color trackbars
        cv2.createTrackbar("Low_H2", self.hsv_window, 0, 255, hsv_track)
        cv2.createTrackbar("Low_S2", self.hsv_window, 29, 255, hsv_track)
        cv2.createTrackbar("Low_V2", self.hsv_window, 102, 255, hsv_track)
        cv2.createTrackbar("Up_H2", self.hsv_window, 55, 255, hsv_track)
        cv2.createTrackbar("Up_S2", self.hsv_window, 255, 255, hsv_track)
        cv2.createTrackbar("Up_V2", self.hsv_window, 255, 255, hsv_track)

        # Gaussian Blur trackbar
        cv2.createTrackbar("GaussianBlur", self.preprocess_window, 23, 51, blur_track)

        # Canny Edge Detection trackbars
        cv2.createTrackbar("CannyLow", self.preprocess_window, 68, 255, canny_track)
        cv2.createTrackbar("CannyHigh", self.preprocess_window, 100, 255, canny_track)

        hsv_track(0)
    

    def crop_img(self, cv_img, edges):
        # 특정 이미지 추출 (전체 높이의 1/10 이미지 )
        height, width = cv_img.shape[:2]
        image_height = height // 10 
        cropped_edges = edges[height - image_height:height, :]
        return cropped_edges, height, width, image_height

    def run(self):   
        if self.cam_flag:
            img_msg = self.img_msg
            cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg) # OpenCV 이미지로 변환
            
            cvt_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            # White color mask
            lower1 = np.array([self.L_H_Value1, self.L_S_Value1, self.L_V_Value1])
            upper1 = np.array([self.U_H_Value1, self.U_S_Value1, self.U_V_Value1])
            mask1 = cv2.inRange(cvt_hsv, lower1, upper1)
            
            # Yellow color mask
            lower2 = np.array([self.L_H_Value2, self.L_S_Value2, self.L_V_Value2])
            upper2 = np.array([self.U_H_Value2, self.U_S_Value2, self.U_V_Value2])
            mask2 = cv2.inRange(cvt_hsv, lower2, upper2)

            # Combine masks
            mask = cv2.bitwise_or(mask1, mask2)
            hsv_img = cv2.bitwise_and(cv_img, cv_img, mask=mask)

            # 트랙바 값 가져오기
            self.GaussianBlur_Value = cv2.getTrackbarPos('GaussianBlur', self.preprocess_window)
            self.CannyLow_Value = cv2.getTrackbarPos('CannyLow', self.preprocess_window)
            self.CannyHigh_Value = cv2.getTrackbarPos('CannyHigh', self.preprocess_window)

            gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
            # 가우시안 블러 필터 적용
            blurred_img = cv2.GaussianBlur(gray, (self.GaussianBlur_Value, self.GaussianBlur_Value), 0)
            # Canny 엣지 검출
            edges = cv2.Canny(blurred_img, self.CannyLow_Value, self.CannyHigh_Value)

            # crop_img 메소드 호출
            cropped_edges, height, width, image_height = self.crop_img(cv_img, edges)

            # 이미지 초기화 
            center_x = width // 2
            threshold_count = 10  # 설정한 픽셀 최소값 임계치
            line_center_left = 0  # 왼쪽 점 초기화
            line_center_right = width  # 오른쪽 점 초기화
            max_white_count_left = 0 # 왼쪽 흰색 픽셀의 최대 수
            max_white_count_right = 0 # 오른쪽 흰색 픽셀의 최대 수

            # 왼쪽 영역에서 검출
            for x in range(0, center_x, 10):
                white_count = cv2.countNonZero(cropped_edges[:, x:x+10])
                if white_count > max_white_count_left:
                    max_white_count_left = white_count
                    line_center_left = x + 5

            # 오른쪽 영역에서 검출
            for x in range(center_x, width, 10):
                white_count = cv2.countNonZero(cropped_edges[:, x:x+10])
                if white_count > max_white_count_right:
                    max_white_count_right = white_count
                    line_center_right = x + 5

            # 충분한 픽셀이 검출되지 않으면 영역의 끝으로 설정
            if max_white_count_left < threshold_count:
                line_center_left = 0  # 왼쪽 끝
            if max_white_count_right < threshold_count:
                line_center_right = width  # 오른쪽 끝

            center_point = (line_center_left + line_center_right) // 2
        
            cv2.circle(cv_img, (line_center_left, height - image_height // 2), 10, (0, 0, 255), -1)  # Red circle at the left line center
            cv2.circle(cv_img, (line_center_right, height - image_height // 2), 10, (255, 0, 0), -1)  # Blue circle at the right line center
            cv2.circle(cropped_edges, (line_center_left, height - image_height // 2), 10, (255, 255, 255), -1)  # 원을 흰색으로 'lane_detection' 창에 그리기
            cv2.circle(cropped_edges, (line_center_right, height - image_height // 2), 10, (255, 255, 255), -1)  # 원을 흰색으로 'lane_detection' 창에 그리기

            # 중간점에 하얀색 점 그리기
            cv2.circle(cv_img, (center_point, height - image_height // 2), 10, (255, 255, 255), -1)  # White circle at the middle point
            cv2.circle(cropped_edges, (center_point, height - image_height // 2), 10, (255, 255, 255), -1)

            ############## 이미지 출력을 위한 코드 ###################
            cv2.imshow('Lane Detection', cropped_edges)
            cv2.imshow('Original Frame', cv_img)
            cv2.waitKey(1)
            #######################################################
            # run 함수가 일정한 주기로 실행
            self.rate.sleep()
            
def main(): 
    try:
        image_processor = Lane_detect()
        while not rospy.is_shutdown():
            image_processor.run()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()  
