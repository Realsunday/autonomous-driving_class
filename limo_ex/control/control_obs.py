#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import math
from geometry_msgs.msg import Twist

class move_limo:
    def __init__(self):
        rospy.init_node('control')
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # 메시지 발행 주기를 설정합니다. 이 예제에서는 10Hz로 설정합니다.
        # rospy.Subscriber('obstacle', String, self.obstacle_fn)
        rospy.Subscriber('obstacle_dist', Int32, self.obstacle_dist_fn)
        self.obs_dist = 0
        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션): 발행 주기 설정
        
    def obstacle_dist_fn(self,data):
        self.obs_dist = data.data
        print("obstacle_dist:",self.obs_dist)

    def drive_control(self):

            try:
                #rospy.loginfo("off_center, lateral_gain = {}, {}".format(self.off_center, self.LATERAL_GAIN))
                self.speed = 0.5
                self.angle = 0.0
                drive = Twist()
                drive.linear.x = self.speed
                drive.angular.z = self.angle

                if  self.obs_dist < 20:
                    drive.linear.x =0
                    self.drive_pub.publish(drive)
                    print("stop")
                elif self.obs_dist < 50:
                    drive.linear.x = 0.1
                    self.drive_pub.publish(drive)
                    print("speed down")
                else:
                    self.drive_pub.publish(drive)

                #self.drive_pub.publish(drive)
                
                # 지정한 발행 주기에 따라 슬립합니다. 이것은 메시지를 일정한 주기로 발행하기 위해 사용됩니다.
                self.rate.sleep()  # ROS 3-1단계(옵션): 퍼블리셔 - 주기 실행
            except Exception as e:
                print("error:",e)
    def obstacle_fn(self,data):
        self.is_obs = data.data
        print(self.is_obs)
                                 

if __name__ == '__main__':
    MoveCar = move_limo()
    try:
        while not rospy.is_shutdown():
            MoveCar.drive_control()
    except KeyboardInterrupt:
        print("program down")
