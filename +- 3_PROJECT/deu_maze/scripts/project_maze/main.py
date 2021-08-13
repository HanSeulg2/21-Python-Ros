#!/usr/bin/env python
# _*_ coding: UTF-8 _*_
import rospy
import Findway
import botinfo

rospy.init_node('main_node')

'''
es = Findway.Begin()

if __name__ == "__main__":
    es.get_target()
'''

# rospy.init_node('deu_maze')
maze_run = botinfo.Run()

if __name__ == "__main__":
    maze_run.get_fin()

