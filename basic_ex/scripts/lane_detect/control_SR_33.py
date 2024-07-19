#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class move_limo:
    def __init__(self):
        rospy.init_node('control')

        self.BASE_ANGLE = 0
        self.BASE_SPEED = 0.8
        self.left_x = 0
        self.KP = 0.01
        self.white_ratio = 0
        
        rospy.Subscriber("/left_x", Int32, self.lane_cb)
        rospy.Subscriber("/white_ratio", Int32, self.white_ratio_cb)
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("obstacle_dist", Int32, self.obstacle_dist_fn)
        self.obs_dist = 100
        self.rate = rospy.Rate(20)

    def obstacle_dist_fn(self, data):
        self.obs_dist = data.data
        print("obstacle_dist:", self.obs_dist)

    def white_ratio_cb(self, data):
        self.white_ratio = data.data
        print("white_ratio:", self.white_ratio)

    def drive_control(self):
        try:
            # print(f"Driving control called, white_ratio: {self.white_ratio}, obs_dist: {self.obs_dist}, left_x: {self.left_x}")
            self.BASE_ANGLE = self.KP*(130 - self.left_x)        # 50 변경해야 함
            print(f"Calculated BASE_ANGLE: {self.BASE_ANGLE}")

            drive = Twist()
            drive.linear.x = self.BASE_SPEED
            drive.angular.z = self.BASE_ANGLE

            if self.white_ratio >= 20:
                drive.linear.x = 2
                drive.angular.z = 0
            else:
                drive.linear.x = self.BASE_SPEED  # white_r        self.BASE_SPEED = 2atio가 50 미만일 때 속도 저하 , 

            if self.obs_dist < 30:
                drive.linear.x = 0
                print("stop")
            elif self.obs_dist < 100:
                drive.linear.x = max(drive.linear.x * 0.3, 0)  # 장애물 근접 시 추가 속도 저하
                print("speed down")
            
            self.drive_pub.publish(drive)
            self.rate.sleep()
        except Exception as e:
            print("error:", e)

    def lane_cb(self, data):
        if data.data == 0:
            self.left_x = 0
        else:
            self.left_x = data.data
        print(self.left_x)

if __name__ == '__main__':
    MoveCar = move_limo()
    try:
        while not rospy.is_shutdown():
            MoveCar.drive_control()
    except KeyboardInterrupt:
        print("program down")