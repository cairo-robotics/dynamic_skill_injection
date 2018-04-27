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
TN_INFO_PUBLISHER = None
TASK_COMMAND_PUBLISHER = None

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

def execute_plan(plan):
	# For task primitive in plan:
	#    Publish task_command for execution
	#    Determine expected changes to world state as a result of primitive
	#    Publish task_network_info
	#    Wait for report from FailureNotifier
	#    If primitive succeeded:
	#        continue to execute next primitive operator
	#    Else:
	#        While the FailureMapper has recommended resolution steps available:
	#            Execute resolution step
	#            Retry failed primitive operator


def world_state_in(msg):
	global STATE
	STATE = json.loads(msg.data)

def command_in(msg):
	# TODO: Implement
	# Parse command and parameters.  Then run PyHOP to get a plan (i.e. list of sequenced primitive operators to execute)

	# Execute the plan
	execute_plan(plan)

def init_listeners():
	# TODO: Need to ensure these message names, data types, etc. are correct
	rospy.Subscriber("/dsi/user_commands", )
	rospy.Subscriber("/dsi/current_world_state", String, world_state_in)
	rospy.Subscriber("/dsi/failure_notification")
	rospy.Subscriber("/dsi/failure_resolution")

def create_publishers():
	global TN_INFO_PUBLISHER
	global TASK_COMMAND_PUBLISHER
	# TODO: Need to make sure the message names, data types, etc. are correct
	TN_INFO_PUBLISHER = rospy.Publisher('task_network_info', , )
	TASK_COMMAND_PUBLISHER = rospy.Publisher('task_command', , )


if __name__ == '__main__':
	main()
