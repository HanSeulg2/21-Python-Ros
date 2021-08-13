#!/usr/bin/env python
# _*_ coding: UTF-8 _*_
import rospy
import math
import time
from collections import deque
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from botinfo import BotInfo


# 도착 지점으로부터 반환 주행을 하기 위해 최단 경로를 계산하는 파일
# botinfo.py 에서 데이터를 들고와서 실행한 후, main.py는 Findway.py를 실행한다.


class BringSaveData(list):
    def __init__(self):
        self.bi = BotInfo()
        self.data_state = False

    def take_data(self):  # 데이터를 botinfo.py 에서 가져옴
        blockredline = 0  # 빨간줄 없애기 위한 의미없는 코드

    def save_data(self):  # 데이터를 botinfo.py 에 저장
        blocklinered = 0  # 빨간줄 없애기 위한 의미없는 코드


# 시작점부터 종료지점 까지 일단 주행
class GoToStart:
    def __init__(self):
        self.startMaze = False
        self.rotation = False
        self.run_count = 0
        self.turn = Twist()
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.bd = BringSaveData()
        self.alert = False  # 도착알림

    def go_to_start(self):
        redlineblock = 0  # 빨간줄 없애기 위한 의미없는 코드


# 최단 경로 알고리즘 으로 다시 시작점으로 감
class BackTo:
    def __init__(self):
        self.fr = BotInfo()
        self.rate = rospy.Rate(10)
        self.flag_restart = True

        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.fr.scan_callback)
        self.odom_scan = rospy.Subscriber('/odom', Odometry, self.fr.get_odom_data)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.frontier = deque()  # deque 사용
        self.path = []  # 탐색 경로 모든 좌표
        self.visited = set()  # 방문 좌표 objects 형 선언
        self.solution = {}  # 최종 탐색 완료된 최단거리 좌표
        self.cntX = len(self.fr.stack_x)
        self.cntY = len(self.fr.stack_y)
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
            if (self.cntX - self.cntP, self.cntY) in self.path and (self.cntX - self.cntP, self.cntY) not in self.visited:  # check the cell on the left
                cell = (self.fr.stack_x - self.fr.count_point, self.fr.stack_y)
                # solution 배열에 좌표를 넣기위한 cell 변수
                self.solution[cell] = self.fr.stack_x, self.fr.stack_y
                # backtracking routine [cell] is the previous cell. x, y is the current cell
                self.frontier.append(cell)  # add cell to frontier list
                self.visited.add((self.fr.stack_x - self.fr.count_point, self.fr.stack_y))  # add cell to visited list

            if (self.fr.stack_x, self.fr.stack_y - self.fr.count_point) in self.path and (self.fr.stack_x, self.fr.stack_y - self.fr.count_point) not in self.visited:  # check the cell down
                cell = (self.fr.stack_x, self.fr.stack_y - self.fr.count_point)
                self.solution[cell] = self.fr.stack_x, self.fr.stack_y
                self.frontier.append(cell)
                self.visited.add((self.fr.stack_x, self.fr.stack_y - self.fr.count_point))
                print(self.solution)

            if (self.fr.stack_x + self.fr.count_point, self.fr.stack_y) in self.path and (self.fr.stack_x + self.fr.count_point, self.fr.stack_y) not in self.visited:  # check the cell on the  right
                cell = (self.fr.stack_x + 24, self.fr.stack_y)
                self.solution[cell] = self.fr.stack_x, self.fr.stack_y

                self.frontier.append(cell)
                self.visited.add((self.fr.stack_x + 24, self.fr.stack_y))

            if (self.fr.stack_x, self.fr.stack_y + 24) in self.path and (self.fr.stack_x, self.fr.stack_y + 24) not in self.visited:  # check the cell up
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

"""
BFS Algorithm
def printing():
    val = BotInfo.get_odom_data()

    print ' x = %.2f y = %.2f' % (val.x, val.y)



class BFS:
    def __init__(self):
        pass
    
    def search(self, gridworld, begin, end):
        global frontier
        global start, goal, explored, cost

        start = begin;
        goal = end
        frontier.put(start)

        explored[start] = None
        cost[start] = 0

        gridworld.mark(start)
        gridworld.markpath(start)

        if self.proceed(gridworld) == 1:
            self.makepath()
            return False
        else:
            self.proceed(gridworld)
            return True

    def proceed(self, gridworld):
        global frontier
        global start, goal, explored, cost

        if frontier.isEmpty():
            return 1

        else:
            current = frontier.get()

            if current == goal:
                return 1
            # sys.exit()
            for next in gridworld.get8Neighbors(current):
                if next not in cost:
                    if next[0] == current[0] or next[1] == current[1]:
                        cost[next] = cost[current] + 5
                    else:
                        cost[next] = cost[current] + 7
                    frontier.put(next)
                    gridworld.mark(next)
                    explored[next] = current
        return 0

    def makepath(self, gridworld):
        global goal, explored, cost, start
        path = []
        current = goal
        while current != start:
            path.append(current)
            gridworld.markpath(current)
            current = explored[current]
        path.reverse();
        path = [start] + path
        return path, explored, cost
"""
