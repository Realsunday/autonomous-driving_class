#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import math

class Obs_detect:
    def __init__(self):
        rospy.init_node("obs_detect_node")
        rospy.loginfo("노드 초기화 완료")

        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)
        rospy.loginfo("/scan 토픽 구독 초기화 완료")

        self.pub_dist90_R = rospy.Publisher("/dist90R", Float32, queue_size=1)
        self.pub_dist75_R = rospy.Publisher("/dist75R", Float32, queue_size=1)
        self.pub_dist90_L = rospy.Publisher("/dist90L", Float32, queue_size=1)
        self.pub_dist75_L = rospy.Publisher("/dist75L", Float32, queue_size=1)
        self.pub_obs_R = rospy.Publisher("/obsR", Bool, queue_size=1)
        self.pub_obs_L = rospy.Publisher("/obsL", Bool, queue_size=1)

        self.laser_msg = LaserScan()
        self.rate = rospy.Rate(5)
        self.laser_flag = False
        self.degrees = []
        self.degrees_flag = False

    def lidar_CB(self, msg):
        if msg:
            self.laser_msg = msg
            self.laser_flag = True
            #rospy.loginfo("Laser scan 데이터 수신됨")
        else:
            self.laser_flag = False

    def sense(self):
        if not self.laser_flag:
            return

        current_laser = self.laser_msg

        pub_dist90_R_list = []
        pub_dist90_L_list = []
        pub_dist75_R_list = []
        pub_dist75_L_list = []
        pub_obs_R_list = []
        pub_obs_L_list = []

        if len(current_laser.ranges) > 0:
            if not self.degrees_flag:
                for i, v in enumerate(current_laser.ranges):
                    self.degrees.append((current_laser.angle_min + current_laser.angle_increment * i) * 180 / math.pi)
                self.degrees_flag = True
                #rospy.loginfo("각도 계산 완료")

            if self.degrees_flag:
                for i, n in enumerate(current_laser.ranges):
                    x = n * math.cos(self.degrees[i] * math.pi / 180)
                    y = n * math.sin(self.degrees[i] * math.pi / 180)
                    
                    if  0.1 < y < 0.45 and 0 < x < 0.4:
                        pub_obs_L_list.append(n)
                    if -0.45 < y < -0.1 and 0 < x < 0.4:
                        pub_obs_R_list.append(n)
                    if 0 < n < 0.5 and -91 < self.degrees[i] < -89:
                        pub_dist90_R_list.append(n)
                    if 0 < n < 0.5 and -76 < self.degrees[i] < -74:
                        pub_dist75_R_list.append(n)
                    if 0 < n < 0.5 and 89 < self.degrees[i] < 91:
                        pub_dist90_L_list.append(n)
                    if 0 < n < 0.5 and 74 < self.degrees[i] < 76:
                        pub_dist75_L_list.append(n)

                #rospy.loginfo("거리 및 장애물 감지 결과를 퍼블리싱합니다.")

                if pub_dist90_R_list:
                    self.pub_dist90_R.publish(sum(pub_dist90_R_list) / len(pub_dist90_R_list))
                else:
                    self.pub_dist90_R.publish(0.3)

                if pub_dist75_R_list:
                    self.pub_dist75_R.publish(sum(pub_dist75_R_list) / len(pub_dist75_R_list))
                else:
                    self.pub_dist75_R.publish(0.3)

                if pub_dist90_L_list:
                    self.pub_dist90_L.publish(sum(pub_dist90_L_list) / len(pub_dist90_L_list))
                else:
                    self.pub_dist90_L.publish(0.3)

                if pub_dist75_L_list:
                    self.pub_dist75_L.publish(sum(pub_dist75_L_list) / len(pub_dist75_L_list))
                else:
                    self.pub_dist75_L.publish(0.3)

                if len(pub_obs_R_list) > 1:
                    self.pub_obs_R.publish(True)
                    print("obs_R:", True)
                else:
                    self.pub_obs_R.publish(False)
                    print("obs_R:", False)

                if len(pub_obs_L_list) > 1:
                    self.pub_obs_L.publish(True)
                    print("obs_L:", True)
                else:
                    self.pub_obs_L.publish(False)
                    print("obs_L:", False)

def main():
    obs_detect = Obs_detect()
    while not rospy.is_shutdown():
        obs_detect.sense()
        obs_detect.rate.sleep()

if __name__ == "__main__":
    main()
