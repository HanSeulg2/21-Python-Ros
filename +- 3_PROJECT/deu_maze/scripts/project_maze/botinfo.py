#!/usr/bin/env python
# _*_ coding: UTF-8 _*_
import rospy
import math
import time
from collections import deque
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class BotInfo:
    def __init__(self):
        self.x = self.y = 0.0

        self.finX = 15.80
        self.finY = -9.7

        self.range_ahead = self.range_left = self.range_right = 0
        self.target_angle = 0

        self.stack_x = []
        self.stack_x = list(map(float, self.stack_x))
        self.stack_y = []
        self.stack_y = list(map(float, self.stack_y))
        self.location = []  # 막힌 길 좌표

        self.stack_x1 = []
        self.stack_x1 = list(map(float, self.stack_x))
        self.stack_y1 = []
        self.stack_y1 = list(map(float, self.stack_y))
        self.all_location = []  # 회전한 위치 좌표

        self.state_drive = False
        self.state_rotate = False
        self.flag_start = True
        self.state_fin = False

        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.count_point = 0
        self.count_point1 = 0
        self.frontier = deque()
        self.solution = []

        self.roll = self.pitch = self.yaw = 0.0

        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_scan = rospy.Subscriber('/odom', Odometry, self.get_odom_data)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # front
        self.range_left = msg.ranges[len(msg.ranges) / 4 * 3]  # left
        self.range_right = msg.ranges[len(msg.ranges) / 4]  # right

        # 로봇 센서값 확인
        '''print 'ahead = %.2f left = %.2f right = %.2f yaw = %.2f x = %.2f y = %.2f' % (self.range_ahead,
                                                                                      self.range_left,
                                                                                      self.range_right,
                                                                                      math.degrees(self.yaw)
                                                                                      , self.x, self.y)'''

    # 지도로부터 로봇의 현재 좌표 및 보고있는 각도 정보를 get
    def get_odom_data(self, msg):
        orientation_value = msg.pose.pose.orientation
        orientation_list = [orientation_value.x, orientation_value.y, orientation_value.z, orientation_value.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def save_xy(self):
        avg = (self.range_ahead + self.range_left + self.range_right) / 3
        if self.state_rotate is True:
            self.stack_x1.append(self.x)
            self.stack_y1.append(self.y)
            self.count_point1 += 1

        if avg < 0.86 and self.state_rotate is True:
            self.stack_x.append(self.x)
            self.stack_y.append(self.y)
            self.count_point += 1

            # x, y 좌표 확인 코드
            # print self.stack_x[self.count_point - 1], self.stack_y[self.count_point - 1]

    def first_escape(self):
        if self.flag_start is True:
            for i in range(9):
                self.twist.angular.z = -math.degrees(1.5708)
                self.pub.publish(self.twist)

                if math.degrees(self.yaw) == -90:
                    self.rate.sleep()
                    self.flag_start = False
                    break

        else:
            if self.state_rotate is True:
                for i in range(10):
                    self.pub.publish(self.twist)
                    self.rate.sleep()
                self.state_rotate = False

            else:
                if self.state_drive is True:
                    if self.range_ahead < 0.7:
                        self.state_drive = False

                    elif (4.40 < self.range_ahead < 4.50) and (self.range_left > self.range_right) and (math.degrees(
                            self.yaw) == -90):
                        self.state_drive = False

                    elif (4.40 < self.range_ahead < 4.50) and (self.range_left > self.range_right) and (
                                    math.degrees(self.yaw) == -90):
                        self.state_drive = False

                    elif (1.00 < self.range_ahead < 1.10) and (4.10 < self.range_left < 4.20) and \
                            (2.50 < self.range_right < 2.60) and (math.degrees(self.yaw) == 0):
                        self.target_angle = -90
                        self.rotating()

                    elif (2.90 < self.range_ahead < 2.95) and (0.60 < self.range_left < 0.70) and \
                            (2.40 < self.range_right < 2.50) and (math.degrees(self.yaw) == 90):
                        self.target_angle = -90
                        self.rotating()

                    elif (0.75 < self.range_ahead < 0.80) and (4.20 < self.range_left < 4.30) and \
                            (2.40 < self.range_right < 2.45) and (math.degrees(self.yaw) == 180):
                        self.target_angle = 90
                        self.rotating()

                    elif (0.60 < self.range_ahead < 0.80) and (2.55 < self.range_left < 2.60) and \
                            (2.45 < self.range_right < 2.55) and (math.degrees(self.yaw) == 0):
                        self.target_angle = -90
                        self.rotating()

                    elif (0.60 < self.range_ahead < 0.80) and (4.10 < self.range_left < 4.15) and \
                            (2.40 < self.range_right < 2.45) and (math.degrees(self.yaw) == -90):
                        self.target_angle = -180
                        self.rotating()

                    elif self.state_fin is True:
                        self.state_drive = False

                else:
                    if self.range_ahead > 0.7:
                        self.state_drive = True
                twist = Twist()
                if self.state_drive is True:
                    twist.linear.x = 1.0
                    if self.range_left > self.range_right:
                        self.twist.angular.z = math.degrees(1.5708)
                    elif self.range_left < self.range_right:
                        self.twist.angular.z = -math.degrees(1.5708)
                else:
                    self.state_rotate = True
                self.pub.publish(twist)
                self.rate.sleep()

    def rotating(self):
        self.state_drive = False
        for i in range(9):
            self.twist.angular.z = -math.degrees(1.5708)
            self.pub.publish(self.twist)

            if math.degrees(self.yaw) == self.target_angle:
                self.rate.sleep()
                self.state_drive = True
                break

    def map_xy(self):
        self.stack_x1 = [round(self.stack_x1[i], 2) for i in range(self.count_point1)]
        self.stack_y1 = [round(self.stack_y1[i], 2) for i in range(self.count_point1)]
        self.all_location = zip(self.stack_x1, self.stack_y1)

        self.stack_x = [round(self.stack_x[i], 2) for i in range(self.count_point)]
        self.stack_y = [round(self.stack_y[i], 2) for i in range(self.count_point)]
        self.location = zip(self.stack_x, self.stack_y)

        if self.state_fin is True:
            print self.all_location
            print '\n'
            print self.location
            return 0


class Run:
    def __init__(self):
        self.fr = BotInfo()
        self.sr = BackTo()

    def get_fin(self):
        while not rospy.is_shutdown():
            if(self.fr.finX - 0.1 < self.fr.x < self.fr.finX + 0.1) and (self.fr.finY - 0.1 < self.fr.y <
                                                                         self.fr.finY + 0.1):
                self.fr.state_fin = True
                self.fr.map_xy()
                self.sr.sec_escape()
                break
            else:
                self.fr.first_escape()
                self.fr.save_xy()


class BackTo:
    def __init__(self):
        self.fr = BotInfo()
        self.rate = rospy.Rate(10)
        self.flag_restart = True  # return start
        self.go_back = True
        self.re_rotation = False
        self.j = 0

        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.fr.scan_callback)
        self.odom_scan = rospy.Subscriber('/odom', Odometry, self.fr.get_odom_data)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    # def path_finder(self):

    def sec_escape(self):
        if self.flag_restart is True:
            for i in range(17):
                twist = Twist()
                twist.angular.z = math.degrees(1.5708)
                self.pub.publish(twist)
                self.rate.sleep()
                self.flag_restart = False
        '''
        else:
            twist = Twist()
            '''


