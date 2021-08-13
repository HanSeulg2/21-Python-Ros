#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import sys
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Vacuum: #주 실행함수
    def __init__(self):
        self.fw = FollowingWall()
        self.vc = VacuumClean()
        self.goal = False
    def vac_start(self):
        while not rospy.is_shutdown():
            if self.vc.clean is True:
                print('소요시간 : ', self.fw.t-rospy.Time.now())
                print('청소 면적 : ')
                print('청소가 완료되었습니다, 종료합니다.')
                break
            else:
                if self.fw.fin is False: # 벽타기 완료 전
                    self.fw.following()
                    self.fw.data()

                elif self.fw.is_fin > 0 or self.fw.fin is True : # 벽타기 완료 후, 청소 시작
                    self.vc.set_angle()

class FollowingWall: #벽타기 클래스
    is_fin = 0

    def __init__(self):
        # twist & Goal
        self.driving_forward = True
        self.turn_r = False
        self.turn_l = False
        self.record = False
        self.rotate = False
        self.first_check = True
        self.turn = Twist()
        self.twist = Twist()
        self.fin = False

        # scan(인식)
        self.range_ahead = 1  # 정면
        self.range_left = 1  # 왼쪽
        self.range_right = 1  # 오른쪽

        # odom(터틀봇의 현재좌표)
        self.odom_x = 0
        self.odom_y = 0

        # topic
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)  # odom
        self.odom_scan = rospy.Subscriber('/odom', Odometry, self.get_odom_data)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.roll = self.pitch = self.yaw = 0.0

        self.vc = VacuumClean()

        self.dot_x = []
        self.dot_y = []

    def get_odom_data(self, msg): #현재 orientation 좌표 받아서 roll,pitch,yaw 저장
        orientation_value = msg.pose.pose.orientation
        orientation_list = [orientation_value.x, orientation_value.y, orientation_value.z, orientation_value.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    # function scan
    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # 900 ahead
        self.range_left = msg.ranges[int(len(msg.ranges) / 1.3)]  # 1350 left
        self.range_right = msg.ranges[len(msg.ranges) / 4]  # 450 right

    # function odom
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.xmap = self.odom_x + 11.0 # 내가 지정한 터틀봇 좌표 더하기
        self.ymap = self.odom_y + 9.0

    # 회전시에 좌표값을 저장하는 기능
    def data(self):
        if self.record is True:  # 회전 한다면
            self.dot_x.append(self.xmap)
            self.dot_y.append(self.ymap)
            print('회전 좌표값 저장')
            self.record = False


    # 벽타기시 사용하는 기능
    def following(self):
        # 처음 미로에 진입했을때 아래 조건문 실행
        #터틀봇 몸체의 지름이 0.28로 추정되어 반지름인 0.14에서 오차를 조금 남긴 값을 범위로 주어 충돌을 감지
        if (self.range_ahead < 0.15) or (self.range_left < 0.15) or (self.range_right<0.15):
            print('충돌! 시스템을 종료합니다')
            sys.exit()
        if self.first_check is True:
            self.t = rospy.Time.now()
            print('청소를 시작합니다')
            # 90도 회전해서 미로에 진입
            for i in range(10):  # turn 180
                self.twist.angular.z = -math.degrees(1.5708)
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)

                if math.degrees(self.yaw) == -90.00:
                    break
            first_move = Twist()
            first_move.linear.x = 0.5
            self.cmd_vel_pub.publish(first_move)
            rospy.sleep(5)
            self.first_check = False
        else:
            # 벽타기가 끝나고 시작 위치로 돌아오면
            if ((abs(self.x) <= 0.4) and (abs(self.y) <= 0.4)) and self.driving_forward is False:
                self.fin = True
                is_fin = 1
                print(is_fin)
                print('fin')
                print('x : ', self.dot_x)
                print('y : ', self.dot_y)
                self.vc.set_angle()
            else:
                if self.rotate is True:
                    for i in range(10):
                        self.cmd_vel_pub.publish(self.turn)
                        self.rate.sleep()
                    self.rotate = False
                else:
                    if self.driving_forward is True:
                        if self.range_ahead <= 0.4:  # 정면거리가 0.4 이하이면 주행을 멈춰라->rotate 수행
                            self.driving_forward = False
                            print('회전 좌표값', self.xmap, self.ymap)
                        elif self.range_left > 1.0:  # 정면거리가 0.4 이하이고 좌측이 안 막혀있으면 정지
                            self.driving_forward = False
                            self.turn_l = True
                            print('회전 좌표값', self.xmap, self.ymap)
                    else:
                        if self.range_ahead > 0.1:
                            self.driving_forward = True
                    twist = Twist()
                    # driving_forward = True 이면 선속도 0.4로 직진하고 아닐경우에 조건에 맞게 회전을 수행
                    if self.driving_forward is True:
                        twist.linear.x = 0.4
                        # 회전시 왼쪽과 오른쪽 인식값 중 더 먼지점으로 회전 수행( 거리가 먼 벽쪽으로 회전)
                        if self.range_left < self.range_right:
                            self.turn.angular.z = -math.degrees(1.5708)
                    else: # 트인 벽 돌기
                        if self.turn_l is True:
                            self.record = True
                            self.driving_forward = True
                            self.turn_l = False
                            for i in range(4):
                                turn = Twist()
                                turn.angular.z = math.degrees(1.5708)
                                self.cmd_vel_pub.publish(turn)
                                rospy.sleep(1)
                                if i is 1:
                                    a = 3
                                else:
                                    a = 1
                                for i in range(a):
                                    gost = Twist()
                                    gost.linear.x = 0.5
                                    self.cmd_vel_pub.publish(gost)
                                    rospy.sleep(1)
                        else:
                            self.rotate = True
                            self.record = True
                    self.cmd_vel_pub.publish(twist)
                    self.rate.sleep()


class VacuumClean: #청소 클래스
    def __init__(self):
        # twist & Goal
        self.driving_forward = True  # 직진 여부 판단

        self.roll = self.pitch = self.yaw = 0.0

        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_scan = rospy.Subscriber('/odom', Odometry, self.get_odom_data)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.turn_r = True
        self.x = self.y = 0.0

        self.range_ahead = self.range_left = self.range_right = 0

        self.stack_x = []
        self.stack_x = map(float, self.stack_x)
        self.stack_y = []
        self.stack_y = map(float, self.stack_y)

        self.clean = True
        self.state_drive = True
        self.state_rotate = False
        self.flag_start = True

        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.count_point = 0

        self.cnt = 0
        self.angle = [-90, -180, 90, 0]

        self.fw = FollowingWall()

    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # front
        self.range_left = msg.ranges[len(msg.ranges) / 12 * 3]  # left
        self.range_right = msg.ranges[len(msg.ranges) / 30 ]  # right

        print ('ahead = %.2f left = %.2f right = %.2f yaw = %.2f x = %.2f y = %.2f' % (self.range_ahead,self.range_left, self.range_right, math.degrees(self.yaw) , self.x, self.y))

    # 지도로부터 로봇의 현재 좌표 및 보고있는 각도 정보를 get
    def get_odom_data(self, msg):
        orientation_value = msg.pose.pose.orientation
        orientation_list = [orientation_value.x, orientation_value.y, orientation_value.z, orientation_value.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def only_turn(self): # 하드코딩의 산출물, 필요한 지점까지 돌면서 이동
        for m in range(10):  # turn 180
            self.twist.angular.z = -math.degrees(1.5708)
            self.pub.publish(self.twist)
            rospy.sleep(1)

            if math.degrees(self.yaw) == -180.00:
                self.flag_start = False
                break
        for i in range(4):
            while self.range_ahead >= 0.7:
                self.twist.linear.x = 0.2
                self.pub.publish(self.twist)

            for j in range(10):  # turn 180
                self.twist.angular.z = -math.degrees(1.5708)
                self.pub.publish(self.twist)
                rospy.sleep(1)

                if math.degrees(self.yaw) == self.angle[i-1]:
                    self.flag_start = False
                    break

            for k in range(10):  # turn 180
                self.twist.angular.z = -math.degrees(1.5708)
                self.pub.publish(self.twist)
                rospy.sleep(1)

                if math.degrees(self.yaw) == 90.00:
                    self.flag_start = False
                    break
        self.clean = True
        self.set_angle()

    def set_angle(self): # 얘가 왔다리 갔다리
        if self.flag_start is True:
            for i in range(10): # turn 180
                self.twist.angular.z = -math.degrees(1.5708)
                self.pub.publish(self.twist)
                rospy.sleep(1)

                if math.degrees(self.yaw) == -180.00:
                    self.flag_start = False
                    break

        else:
            if self.clean is True:
                self.fw.following()
            else:
                if self.state_drive is True:
                    if self.range_ahead <= 0.7:
                        self.state_drive = False
                        #if self.range_left <= 0.6 or self.range_right <= 0.6:
                            #self.only_turn()
                else:
                    if self.range_ahead >= 0.7 :
                        self.state_drive = True
                twist = Twist()
                if self.state_drive is True:
                    twist.linear.x = 0.2
                    self.pub.publish(twist)
                else:
                    if (self.cnt % 2) == 0:
                        for i in range(2):
                            for j in range(2):
                                turning = Twist()
                                str = Twist()
                                turning.angular.z = math.degrees(1.5708)
                                self.pub.publish(turning)
                                rospy.sleep(1)
                            str.linear.x = 0.5
                            self.pub.publish(str)
                            rospy.sleep(1)
                        self.cnt = self.cnt + 1
                        print(self.cnt)
                        self.state_drive = True
                    elif (self.cnt % 2) == 1:
                        for i in range(2):
                            for j in range(2):
                                turning = Twist()
                                str = Twist()
                                turning.angular.z = -math.degrees(1.5708)
                                self.pub.publish(turning)
                                rospy.sleep(1)
                            str.linear.x = 0.5
                            self.pub.publish(str)
                            rospy.sleep(1)
                        self.cnt = self.cnt + 1
                        print(self.cnt)
                        self.state_drive = True

    def cleaning(self):
        twist = Twist()
        if self.turn_r is False:
            for i in range(2):
                turning = Twist()
                str = Twist()
                turning.angular.z = -math.degrees(1.5708)
                str.linear.x = 0.5
                self.pub.publish(turning)
                self.pub.publish(str)
            rospy.sleep(1)
            self.turn_r = True
        else:
            for i in range(2):
                turning = Twist()
                str = Twist()
                turning.angular.z = -math.degrees(1.5708)
                str.linear.x = 0.5
                self.pub.publish(turning)
                self.pub.publish(str)
            self.rate.sleep()
            self.turn_r = False




# main
if __name__ == "__main__":
    rospy.init_node('maze_module')
    scan_move = Vacuum()
    scan_move.vac_start()