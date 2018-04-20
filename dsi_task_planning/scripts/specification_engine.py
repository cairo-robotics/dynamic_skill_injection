#!/usr/bin/env python

'''
Functions for turning numerical representations of objects into predicate statements
'''

import rospy

import sys
import numpy as np
import threading
import time
from ast import literal_eval

from std_msgs.msg import String

#GLOBAL VARIABLES
RETURN_PUBLISHER = None
REQUEST_QUEUE = []def command_in(msg):
	global action_queue
	if(msg.data == "EMERGENCY"):
		override_publisher.publish("HALT")
	else:
		if(movement_active):
			action_queue.append(msg)
		else:
			execute_command(msg)

# pose_publisher = None
# fingers_publisher = None
# override_publisher = None
# sequence_status_publisher = None
# movement_active = False
# action_queue = []
queue_lock = None



def main():
	global queue_lock
	queue_lock = threading.Lock()
	rospy.init_node("abstraction_engine")
	init_listeners()
	create_publishers()
	rospy.spin()


def request_in(msg):
	global REQUEST_QUEUE
	request_type = msg.data.split(':')[0]
	if request_type=='generalization':
		result = generalize(msg.data.split(':')[1])
	elif request_type=='specification':
		result = specify(msg.data.split(':')[1])


	# if(msg.data == "EMERGENCY"):
	# 	override_publisher.publish("HALT")
	# else:
	# 	if(movement_active):
	# 		action_queue.append(msg)
	# 	else:
	# 		execute_command(msg)


def specify(param_list):
	preposition = param_list[0]
	robot_loc = literal_eval(param_list[1])
	obj_dims = literal_eval(param_list[2])
	tgt_type = literal_eval(param_list[3])
	tgt_loc = literal_eval(param_list[4])
	tgt_dims = literal_eval(param_list[5])
	# TODO: These calculations assume object pose is grid-aligned with axes (i.e. no rotation)
	if preposition=='on':
		result = specify_on(robot_loc, obj_dims, tgt_loc, tgt_dims)
	elif preposition=='to':
		result = specify_to(robot_loc, obj_dims, tgt_loc, tgt_dims)
	return result

def specify_on(robot_loc, obj_dims, tgt_loc, tgt_dims):
	# TODO: Make position for placing object more elegant instead of just in the middle of tgt loc
	result = (tgt_loc[0], tgt_loc[1], tgt_loc[2] + (.5 * tgt_dims[2]))
	return result

def specify_to(robot_loc, obj_dims, tgt_loc, tgt_dims):
	# TODO: Need to fix the result so it accounts for robot location
	result = (tgt_loc[0], tgt_loc[1] - tgt_dims[1], robot_loc[2])
	return result



def init_listeners():
	rospy.Subscriber("/sensor_output", String, sensor_in)
	rospy.Subscriber("/abstraction_request", String, request_in)

def create_publishers():
	global RETURN_PUBLISHER
	RETURN_PUBLISHER = ropspy.Publisher('abstraction_return', String, queue_size = 1)



if __name__ == '__main__':
	main()
