#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import numpy as np

class SLAMControl:
    def __init__(self):
        rospy.init_node("slam_control_node")

        # 지도 데이터와 로봇 위치 데이터를 구독
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        self.cmd_msg = Twist()
        self.map_data = None
        self.robot_pose = None
        self.rate = rospy.Rate(2)  # 루프 주기를 2Hz로 설정

    def map_callback(self, data):
        self.map_data = data

    def odom_callback(self, data):
        # 로봇의 현재 위치와 방향을 추출
        self.robot_pose = data.pose.pose

    def control(self):
        if self.map_data is None or self.robot_pose is None:
            return

        # 현재 로봇 위치
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        quaternion = (
            self.robot_pose.orientation.x,
            self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,
            self.robot_pose.orientation.w
        )
        _, _, robot_yaw = tf.transformations.euler_from_quaternion(quaternion)

        # 주행 로직 (간단한 장애물 회피 예제)
        speed = 0.2
        steer = 0.0

        # 맵 데이터와 로봇 위치가 제대로 수신되었는지 확인
        if self.map_data is not None and self.robot_pose is not None:
            # 벽 인식 로직 (간단한 거리 기반 예제)
            # 원하는 방향으로 주행하는 예제
            steer = 0.5  # 오른쪽으로 회전 (예제)

        self.cmd_msg.linear.x = speed
        self.cmd_msg.angular.z = steer
        self.pub.publish(self.cmd_msg)

def main():
    control = SLAMControl()
    while not rospy.is_shutdown():
        control.control()
        control.rate.sleep()

if __name__ == "__main__":
    main()