#!/usr/bin/env python

'''
Node for executing specific actions on the Jaco arm
'''

import rospy
import json
import threading
import time
import ast
import copy

import planning_domain as pd
import pyhop
from std_msgs.msg import String
from dynamic_skill_injection_msgs.msg import TaskNetwork

# GLOBAL VARIABLES
STATE = None
TN_INFO_PUBLISHER = None
TASK_COMMAND_PUBLISHER = None
RESOLUTION_REQUEST_PUBLISHER = None
RESOLUTION_DB_UPDATE_PUBLISHER = None
INITIAL_COMMAND = 'fetch:[mug1, table1, on, desk1]'

def main():
	global queue_lock

	rospy.init_node("planner_executor")
	init_listeners()
	create_publishers()
	pyhop.declare_operators(move_to, open_door, detect, pickup, set_down, unlock_door, turn_on_lights)
	pyhop.declare_methods('navigate_to',navigate_to)
	pyhop.declare_methods('fetch',fetch)
	raw_input("Press Enter to continue...")
	command_in(INITIAL_COMMAND)
	rospy.spin()


def execute_plan(task_name, plan):
	# plan = [('op1', 'arg1', 'arg2'), ('op2', 'arg1', 'arg2')]

	execution_success = True
	for op_and_params, idx in enumerate(plan):
		op_str = op_and_params[0]
		operator = getattr(pd, op_and_params[0])
		op_args = op_and_params[1:]

		# Publish task_command for execution
		# TODO: publish the command

		# Determine expected changes to world state as a result of primitive
		expected_state = copy.deepcopy(STATE)
		operator(expected_state, *op_args)
		expected_effects = get_expected_effects(STATE, expected_state)

		# Publish task_network_info
		tn = TaskNetwork()
		tn.task_name = task_name
		tn.operator = op_str
		tn.effect = expected_effects
		TN_INFO_PUBLISHER.publish(tn)

		# Get response from failure_notification_server
		rospy.wait_for_service('failure_notification_server')
		failure_notifier = rospy.ServiceProxy('failure_notification_server', FailureNotification)
		success = failure_notifier(json.dumps(expected_effects), json.dumps(STATE))

		if success.data=="True":
			continue
		else:
			# Request resolution steps from failure_resolution_server
			rospy.wait_for_service('failure_resolution_server')
			failure_resolver = rospy.ServiceProxy('failure_resolution_server', FailureResolution)
			resolution = failure_resolver(op_str, json.dumps(STATE))
			# If resolution is None:
			if resolution.data=="None":
				# Request recommendation from user
				RESOLUTION_REQUEST_PUBLISHER.publish('run')
				resolution = rospy.wait_for_message("/dsi/human_command", String)
				if resolution.data=="None":
					return False
			resolution_data = json.loads(resolution.data)

			# Publish message to update resolution database
			resolution_update_data = {op_str: resolution_data}
			RESOLUTION_DB_UPDATE_PUBLISHER.publish(json.dumps(resolution_update_data))

			# Parse resolution into input for PyHOP and get plan, called 'action'
			resolution_action_name = resolution_data.keys()[0]
			params = resolution_data['parameterization']
			task = tuple(params.insert(0, resolution_action_name))
			subplan = pyhop.pyhop(STATE, [task], verbose=2)
			# Execute resolution
			if subplan is False:
				print "Error:  Subplan not found."
				return False
			else:
				execute_plan('Failure Resolution: ' + op_str, subplan)

			# Retry failed operator.  Publish task_command for execution
			# TODO: publish the command
	return True

def world_state_in(msg):
	global STATE
	STATE = json.loads(msg.data)

def command_in(msg):
	# Parse command and parameters.
	data_list = msg.split(':')
	task_name = data_list[0]
	task = (task_name)
	operator_args = ast.literal_eval(data_list[1:])
	for arg in operator_args:
		task = task + (arg,)
	# Run PyHOP to get a plan (i.e. list of sequenced primitive operators to execute)
	plan = pyhop.pyhop(STATE, [task], verbose=2)
	# Execute the plan
	if plan is False:
		print "Error:  Plan not found."
	else:
		execute_plan(task_name, plan)

def init_listeners():
	rospy.Subscriber("/dsi/current_world_state", String, world_state_in)

def create_publishers():
	global TN_INFO_PUBLISHER
	global RESOLUTION_REQUEST_PUBLISHER
	global RESOLUTION_DB_UPDATE_PUBLISHER
	# TODO: Need to make sure the message names, data types, etc. are correct
	TN_INFO_PUBLISHER = rospy.Publisher('task_network_info', TaskNetwork)
	RESOLUTION_REQUEST_PUBLISHER = rospy.Publisher('gui_startup', String)
	RESOLUTION_DB_UPDATE_PUBLISHER = rospy.Publisher('/dsi/resolution_actions')

def get_expected_effects(world_state, expected_state):
	expected_effects = {}
	for obj in expected_state:
		expected_effects[obj] = {}
		for attr in obj:
			if type(attr) is list:
				if sorted(world_state[obj][attr])!=sorted(expected_state[obj][attr]):
					expected_effects[obj][attr] = expected_state[obj][attr]
			else:
				if world_state[obj][attr]!=expected_state[obj][attr]:
					expected_effects[obj][attr] = expected_state[obj][attr]
		if not expected_effects[obj]:
			expected_effects.pop(obj)
	return expected_effects

if __name__ == '__main__':
	main()
