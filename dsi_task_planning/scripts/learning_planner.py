#!/usr/bin/env python

'''
Node for executing specific actions on the Jaco arm
'''

import rospy

import sys
import numpy as np
import threading
import time

from std_msgs.msg import String

#GLOBAL VARIABLES
STATE = None
pose_publisher = None
fingers_publisher = None
override_publisher = None
sequence_status_publisher = None
movement_active = False
action_queue = []
queue_lock = None



def main():
	global queue_lock
	queue_lock = threading.Lock()
	rospy.init_node("robot_pointer")
	init_listeners()
	create_publishers()
	rospy.spin()









def init_listeners():
	rospy.Subscriber("/robot_commands", String, command_in)

def create_publishers():
	global pose_publisher
	global fingers_publisher
	global override_publisher
	global sequence_status_publisher
	pose_publisher = rospy.Publisher('kinova_arm_bridge', Pose, queue_size = 1)
	fingers_publisher = rospy.Publisher('kinova_finger_bridge', Point, queue_size = 1)
	override_publisher = rospy.Publisher('kinova_override_bridge', String, queue_size = 1)
	sequence_status_publisher = rospy.Publisher('robot_command_sequence_status', String, queue_size = 1)


if __name__ == '__main__':
	main()
