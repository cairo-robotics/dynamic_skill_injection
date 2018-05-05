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
# FIXME: Should the STATE be locked when a plan is being developed?  How?
STATE = None
TN_INFO_PUBLISHER = None
TASK_COMMAND_PUBLISHER = None
RESOLUTION_REQUEST_PUBLISHER = None
INITIAL_COMMAND = 'fetch:[mug1, table1, on, desk1]'

def main():
	global queue_lock
	queue_lock = threading.Lock()
	rospy.init_node("planner_executor")
	init_listeners()
	create_publishers()
	pyhop.declare_operators(move_to, open_door, detect, pickup, set_down, unlock_door, turn_on_lights)
	pyhop.declare_methods('navigate_to',navigate_to)
	pyhop.declare_methods('fetch',fetch)
	rospy.spin()
	# TODO: Does this go before or after rospy.spin()?
	command_in(INITIAL_COMMAND)

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
		# TODO: publish task network info
		tn = TaskNetwork()
		tn.task_name = task_name
		tn.operator = op_str
		tn.effect = expected_effects
		TN_INFO_PUBLISHER.publish(tn)

		# Get response from failure_notification_server
		exec_time = get_operator_exec_time(op_str)
		# TODO: Change this time delay into wait for message from action_server
		time.sleep(exec_time)
		rospy.wait_for_service('failure_notification_server')
		failure_notifier = rospy.ServiceProxy('failure_notification_server', FailureNotification)
		success = failure_notifier(json.dumps(expected_effects), json.dumps(STATE))

		if success:
			continue
		else:
			# Request resolution steps from failure_resolution_server
			rospy.wait_for_service('failure_resolution_server')
			failure_resolver = rospy.ServiceProxy('failure_resolution_server', FailureResolution)
			resolution = failure_resolver(op_str, json.dumps(STATE))
			# If resolution is None:
			# TODO: Parse resolution to give None or action with parameters
			if resolution is None:
				# Request recommendation from user
				RESOLUTION_REQUEST_PUBLISHER.publish('run')
				resolution = rerospy.wait_for_message("/dsi/human_command", String)
				# TODO: Parse resolution to give None or action with parameters
				if resolution is None:
					return False

			# Parse resolution into input for PyHOP and get plan, called 'action'
			# TODO: Confirm that action is properly parsed for input to PyHOP
			subplan = pyhop.pyhop(STATE, [action], verbose=2)
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
	# TODO: Need to ensure these message names, data types, etc. are correct

	rospy.Subscriber("/dsi/current_world_state", String, world_state_in)

def create_publishers():
	global TN_INFO_PUBLISHER
	global TASK_COMMAND_PUBLISHER
	# TODO: Need to make sure the message names, data types, etc. are correct
	TN_INFO_PUBLISHER = rospy.Publisher('task_network_info', TaskNetwork)
	TASK_COMMAND_PUBLISHER = rospy.Publisher('task_command', , )
	RESOLUTION_REQUEST_PUBLISHER = rospy.Publisher('gui_startup', String)

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

def get_operator_exec_time(op_str):
	exec_time = None
	if op_str=='move_to':
		exec_time = 10.0
	elif op_str=='open_door':
		exec_time = 10.0
	elif op_str=='detect':
		exec_time = 10.0
	elif op_str=='pickup':
		exec_time = 10.0
	elif op_str=='set_down':
		exec_time = 10.0
	elif op_str=='unlock_door':
		exec_time = 10.0
	elif op_str=='turn_on_lights':
		exec_time = 10.0
	elif op_str=='navigate_to':
		exec_time = 10.0
	if exec_time is None:
		print "Error: Unable to calculate operator exec_time"
	return exec_time


if __name__ == '__main__':
	main()
