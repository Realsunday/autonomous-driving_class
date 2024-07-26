#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32
import math
from geometry_msgs.msg import Twist

class move_limo:
    def __init__(self):
        rospy.init_node('control')

        self.BASE_ANGLE = 0
        self.BASE_SPEED = 0.2
        self.left_x = 0
        self.KP = 0.01      # 변경해야 할 값
        
        rospy.Subscriber("/left_x", Int32, self.lane_cb)
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(20)  #20Hz


    def drive_control(self):

            try:
                #rospy.loginfo("off_center, lateral_gain = {}, {}".format(self.off_center, self.LATERAL_GAIN))
                self.BASE_ANGLE = self.KP*(50 - self.left_x)        # 50 변경해야 함
                drive = Twist()
                drive.linear.x = self.BASE_SPEED
                drive.angular.z = self.BASE_ANGLE
                self.drive_pub.publish(drive)
                # 지정한 발행 주기에 따라 슬립합니다. 이것은 메시지를 일정한 주기로 발행하기 위해 사용됩니다.
                self.rate.sleep()  # ROS 3-1단계(옵션): 퍼블리셔 - 주기
            except Exception as e:
                print("error:",e)
                  
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
