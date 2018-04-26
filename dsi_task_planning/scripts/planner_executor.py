#!/usr/bin/env python

'''
Node for executing specific actions on the Jaco arm
'''

import rospy
import json
import threading
import time

from std_msgs.msg import String

#GLOBAL VARIABLES
STATE = None




def main():
	global queue_lock
	queue_lock = threading.Lock()
	rospy.init_node("planner_executor")
	init_listeners()
	create_publishers()
	pyhop.declare_operators(move_to, open_door, detect, pickup, set_down, unlock_door, turn_on_lights, navigate_to)
	pyhop.declare_methods('navigate_to',navigate_to)
	pyhop.declare_methods('fetch',fetch)
	rospy.spin()









def init_listeners():


def create_publishers():



if __name__ == '__main__':
	main()
