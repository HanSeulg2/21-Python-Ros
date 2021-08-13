#!/usr/bin/env python
# _*_ coding: UTF-8 _*_
import rospy
import math
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
        self.stack_x = map(float, self.stack_x)
        self.stack_y = []
        self.stack_y = map(float, self.stack_y)

        self.state_drive = False
        self.state_rotate = False
        self.flag_start = True
        self.state_fin = False

        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.count_point = 0

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
        if self.state_rotate is True:
            self.stack_x.append(self.x)
            self.stack_y.append(self.y)
            self.count_point += 1
            print self.stack_x[self.count_point-1], self.stack_y[self.count_point-1]

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


# 최단 경로 알고리즘 으로 다시 시작점으로 감
class BackTo:
    def __init__(self):
        self.mr = BotInfo()
        self.go_back_state = True  # 다시 시작 지점 가는지 상태
        self.go_back = True
        self.rate = rospy.Rate(10)
        self.re_rotation = False

        self.endpoint = False

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.mr.get_odom_data)  # odom
        self.go_back_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.mr.scan_callback)

    def get_end_point(self): # 첫 주행에서의 좌표를 가져
        while not rospy.is_shutdown():
            # 맵의 목적지 좌표 일정 범위에 들어가면 target = False 바꾸고 멈춰라
            if (-8.55 < self.mr.x < -7.55) and (
                    5.05 < self.mr.y < 5.55) and self.endpoint is True:  # End Range
                break

            # 맵의 출발지 좌표 일정 범위에 들어가면 target = True 바꾸고
            if ((7.75 < self.mr.x < 8.15) and (
                    -5.35 < self.mr.y < -4.95)) or self.endpoint is True:  # Goal Range
                self.endpoint = True
                self.back_to_start()  # 2차 탈출 시작
            else:
                self.mr.first_escape()  # 아닐 경우 1차 탈출
                self.mr.save_xy()  # 좌표 값 저장 not rospy.is_shutdown()

    def back_to_start(self): # 출발지점으로 되돌아 가는 함
        if self.go_back_state is True:  # 2차 탈출 시작하면
            for i in range(21):
                start_maze = Twist()
                start_maze.angular.z = -math.radians(90)  # 180도 회전, 다시 미로 방향으로 튼다
                self.go_back_pub.publish(start_maze)
                self.rate.sleep()
            self.go_back_state = False

        else:
            twist = Twist()
            for i in range(0, BotInfo().count_point, 1):  # 스택에 쌓았던 카운트 개수 만큼 반복
                self.go_back = True
                # 현재 좌표와 스택에 쌍인 좌표 중 0.25 오차만큼의 범위에 들어오면
                if (self.mr.stack_x[i] - 0.25 <= self.mr.x <= self.mr.x[i] + 0.25) and (
                        self.mr.stack_y[i] - 0.25 <= self.mr.y < self.mr.y[i] + 0.25):
                    self.go_back = False  # 직선 주행을 멈추고
                    self.re_rotation = True  # 회전
                    if self.re_rotation is True:  # 회전할 경우
                        if self.go_back is False:
                            # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 멀 경우
                            if self.mr.range_left > self.mr.range_right:
                                for self.j in range(10):
                                    explore = Twist()
                                    explore.angular.z = math.radians(90)  # 왼쪽으로 90도 회전하라
                                    self.go_back_pub.publish(explore)
                                    self.rate.sleep()

                                for i in range(5):
                                    twist.linear.x = 1
                                    self.go_back_pub.publish(twist)
                                    self.rate.sleep()
                                    # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 가까울 경우
                            elif self.mr.range_left < self.mr.range_right:
                                for self.j in range(10):
                                    explore = Twist()
                                    explore.angular.z = -math.radians(90)  # 오른쪽으로 90도 회전하라
                                    self.go_back_pub.publish(explore)
                                    self.rate.sleep()
                                for i in range(5):
                                    twist.linear.x = 1
                                    self.go_back_pub.publish(twist)
                                    self.rate.sleep()
                        self.go_back = True
                        self.re_rotation = False
                else:
                    if self.re_rotation is False:
                        if self.go_back is True:
                            twist.linear.x = 1
            self.go_back_pub.publish(twist)
            self.rate.sleep()

