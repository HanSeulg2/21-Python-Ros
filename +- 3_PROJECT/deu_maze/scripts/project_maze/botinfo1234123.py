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
        """print 'ahead = %.2f left = %.2f right = %.2f yaw = %.2f x = %.2f y = %.2f' % (self.range_ahead,
                                                                                      self.range_left,
                                                                                      self.range_right,
                                                                                      math.degrees(self.yaw)
                                                                                      , self.x, self.y)"""

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


class Run:
    def __init__(self):
        self.fr = BotInfo()
        self.sr = BackTo()

    def get_fin(self):
        while not rospy.is_shutdown():
            if(self.fr.finX - 0.1 < self.fr.x < self.fr.finX + 0.1) and (self.fr.finY - 0.1 < self.fr.y <
                                                                         self.fr.finY + 0.1):
                self.fr.state_fin = True
                self.sr.sec_escape()
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

        self.frontier = deque()  # deque 사용
        self.path = []  # 탐색 경로 모든 좌표
        self.visited = set()  # 방문 좌표 objects 형 선언
        self.solution = {}  # 최종 탐색 완료된 최단거리 좌표
        self.cntX = self.fr.stack_x
        self.cntY = self.fr.stack_y
        self.cntP = self.fr.count_point

    def search_bfs(self):      
        self.frontier.append((self.fr.stack_x, self.fr.stack_y))
        self.solution[self.fr.stack_x, self.fr.stack_y] = self.fr.stack_x, self.fr.stack_y
        self.path.append((self.fr.stack_x, self.fr.stack_y))

        while len(self.frontier) > 0:  # exit while loop when frontier queue equals zero
            time.sleep(0)
            self.fr.stack_x, self.fr.stack_y = self.frontier.popleft()
            # pop next entry in the frontier queue an assign to x and y location
            # stack_x, y 배열을 인덱스 사용 없이 정수형 변수로 배열 탐색이 되려나..?
            # 안되면 for 문으로 인덱스 일일히 탐색해서 노가다 해야지 뭐.
            # 아니면 save_xy 에서 stack 저장을 deque로 해보자
            if (self.cntX - self.cntP, self.cntY) in self.path \
                    and (self.cntX - self.cntP, self.cntY) not in self.visited:
                # check the cell on the left
                cell = (self.fr.stack_x - self.cntP, self.fr.stack_y)
                # solution 배열에 좌표를 넣기위한 cell 변수
                self.solution[cell] = self.fr.stack_x, self.fr.stack_y
                # backtracking routine [cell] is the previous cell. x, y is the current cell
                self.frontier.append(cell)  # add cell to frontier list
                self.visited.add((self.fr.stack_x - self.cntP, self.fr.stack_y))  # add cell to visited list

            if (self.fr.stack_x, self.fr.stack_y - self.cntP) in self.path \
                    and (self.fr.stack_x, self.fr.stack_y - self.cntP) not in self.visited:
                # check the cell down
                cell = (self.fr.stack_x, self.fr.stack_y - self.cntP)
                self.solution[cell] = self.fr.stack_x, self.fr.stack_y
                self.frontier.append(cell)
                self.visited.add((self.fr.stack_x, self.fr.stack_y - self.cntP))
                print(self.solution)

            if (self.fr.stack_x + self.cntP, self.fr.stack_y) in self.path \
                    and (self.fr.stack_x + self.cntP, self.fr.stack_y) not in self.visited:
                # check the cell on the right
                cell = (self.fr.stack_x + self.cntP, self.fr.stack_y)
                self.solution[cell] = self.fr.stack_x, self.fr.stack_y

                self.frontier.append(cell)
                self.visited.add((self.fr.stack_x + 24, self.fr.stack_y))

            if (self.fr.stack_x, self.fr.stack_y + 24) in self.path \
                    and (self.fr.stack_x, self.fr.stack_y + self.cntP) not in self.visited:
                # check the cell up
                cell = (self.fr.stack_x, self.fr.stack_y + 24)
                self.solution[cell] = self.fr.stack_x, self.fr.stack_y

                self.frontier.append(cell)
                self.visited.add((self.fr.stack_x, self.fr.stack_y + 24))

    def sec_escape(self):
        if self.flag_restart is True:
            for i in range(17):
                twist = Twist()
                twist.angular.z = math.degrees(1.5708)
                self.pub.publish(twist)
                self.rate.sleep()
            self.flag_restart = False
        else:
            twist = Twist()
            for i in range(0, self.cntP, 1):  # 스택에 쌓았던 카운트 개수 만큼 반복
                self.go_back = True
                # 현재 좌표와 스택에 쌍인 좌표 중 0.25 오차만큼의 범위에 들어오면
                if (self.fr.stack_x[i] - 0.25 <= self.fr.x <= self.fr.stack_x[i] + 0.25) and (
                        self.fr.stack_y[i] - 0.25 <= self.fr.y < self.fr.stack_y[i] + 0.25):
                    self.go_back = False  # 직선 주행을 멈추고
                    self.re_rotation = True  # 회전
                    if self.re_rotation is True:  # 회전할 경우
                        if self.go_back is False:
                            # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 멀 경우
                            if self.fr.range_left > self.fr.range_right:
                                for self.j in range(10):
                                    explore = Twist()
                                    explore.angular.z = math.radians(90)  # 왼쪽으로 90도 회전하라 degree 변경
                                    self.pub.publish(explore)
                                    self.rate.sleep()

                                for i in range(5):
                                    twist.linear.x = 1
                                    self.pub.publish(twist)
                                    self.rate.sleep()
                                    # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 가까울 경우
                            elif self.fr.range_left < self.fr.range_right:
                                for self.j in range(10):
                                    explore = Twist()
                                    explore.angular.z = -math.radians(90)  # 오른쪽으로 90도 회전하라 degree 변경
                                    self.pub.publish(explore)
                                    self.rate.sleep()
                                for i in range(5):
                                    twist.linear.x = 1
                                    self.pub.publish(twist)
                                    self.rate.sleep()
                        self.go_back = True
                        self.re_rotation = False
                else:
                    if self.re_rotation is False:
                        if self.go_back is True:
                            twist.linear.x = 1
            self.pub.publish(twist)
            self.rate.sleep()
