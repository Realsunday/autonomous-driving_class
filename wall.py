#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi, acos

LEFT = "LEFT"
RIGHT = "RIGHT"

class LimoWallFollowing:
    def __init__(self, direction):
        rospy.init_node("laser_scan_node")
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.cmd_vel_msg = Twist()

        self.is_scan = False
        self.distance = []

        self.condition = None
        self.speed = 0
        self.angle = 0

        self.DIRECTION = direction
        self.default_speed = 0.15
        self.default_angle = 0.2
        self.scan_dist = 0.5
        self.offset = 0.2

    def laser_callback(self, msg):
        # Ensure the incoming message is valid
        if msg and msg.ranges:
            self.msg = msg
            self.is_scan = True
        else:
            rospy.logwarn("Invalid or empty scan data received.")

    def LiDAR_scan(self):
        if hasattr(self, 'msg'):
            try:
                self.degrees = [
                    (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
                    for i, data in enumerate(self.msg.ranges)
                ]
                self.distance = [
                    data for i, data in enumerate(self.msg.ranges)
                    if -90 < self.degrees[i] < 90 and 0 < data <= self.scan_dist
                ]
            except Exception as e:
                rospy.logerr(f"Error processing LIDAR data: {e}")
        else:
            rospy.logwarn("No scan message available for processing.")

    def judge_distance(self):
        if len(self.distance) > 0:
            if min(self.distance) < self.scan_dist - self.offset:
                self.condition = "close"
            elif max(self.distance) > self.scan_dist + self.offset:
                self.condition = "far"
            else:
                self.condition = "maintaining"
        else:
            self.condition = "forward"

    def maintain_direction(self):
        angle1 = self.angle_distance(70 if self.DIRECTION == LEFT else -70)
        angle2 = self.angle_distance(80 if self.DIRECTION == LEFT else -80)
        if angle1 and angle2:
            try:
                theta = acos(min(1, max(-1, angle2 / angle1)))
                if 0 < theta < 0.35:
                    self.angle = theta - self.default_angle if self.DIRECTION == LEFT else -(theta - self.default_angle)
                else:
                    self.angle = -(theta - self.default_angle) if self.DIRECTION == LEFT else (theta - self.default_angle)
                self.speed = self.default_speed
            except ValueError as e:
                rospy.logerr(f"Error in angle calculation: {e}")

    def angle_distance(self, degree):
        if hasattr(self, 'degrees'):
            indices = [
                i for i, degree_val in enumerate(self.degrees)
                if degree < degree_val < degree + 1 and 0 < self.msg.ranges[i] < 0.6
            ]
            if indices:
                return min(self.msg.ranges[i] for i in indices)
        return None

    def move_control(self):
        # Apply movement strategy based on the current condition
        if self.condition == "forward":
            self.speed = self.default_speed
            self.angle = 0
        elif self.condition == "close":
            self.angle = -self.default_angle / (min(self.distance) * 10) if self.DIRECTION == LEFT else self.default_angle / (min(self.distance) * 10)
        elif self.condition == "far":
            self.angle = (self.default_angle) * 2 if self.DIRECTION == LEFT else -(self.default_angle) * 2
        elif self.condition == "maintaining":
            self.maintain_direction()

    def main(self):
        while not rospy.is_shutdown():
            if self.is_scan:
                self.LiDAR_scan()
                self.judge_distance()
                self.move_control()
                self.cmd_vel_msg.linear.x = self.speed
                self.cmd_vel_msg.angular.z = self.angle
                self.pub.publish(self.cmd_vel_msg)
                self.is_scan = False

if __name__ == "__main__":
    direction = LEFT  # Or RIGHT
