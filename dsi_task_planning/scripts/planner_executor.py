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

import task_planning.planning_domain as pd
import task_planning.pyhop as pyhop
from std_msgs.msg import String
from dynamic_skill_injection_msgs.msg import TaskNetwork
from dsi_action_server.srv import action_service
from dynamic_skill_injection_msgs.srv import FailureNotification, FailureResolution


# GLOBAL VARIABLES
STATE = None
TN_INFO_PUBLISHER = None
TASK_COMMAND_PUBLISHER = None
RESOLUTION_REQUEST_PUBLISHER = None
RESOLUTION_DB_UPDATE_PUBLISHER = None
#INITIAL_COMMAND = {'fetch':['cafe_beer', 'blue_5m_room', 'in', 'folding_table_4x2']}
INITIAL_COMMAND = {'move_to':['blue_5m_room']}



def execute_plan(task_name, plan):
	# plan = [('op1', 'arg1', 'arg2'), ('op2', 'arg1', 'arg2')]

	execution_success = True
	for idx,  op_and_params in enumerate(plan):
		op_str = op_and_params[0]
		operator = getattr(pd, op_and_params[0])
		op_args = op_and_params[1:]

		# Publish task_command for execution
		# TODO: publish the command
		print action_server(json.dumps({"method":"open_close_open"}))


		# Determine expected changes to world state as a result of primitive
		expected_state = copy.deepcopy(STATE)
		operator(expected_state, *op_args)
		print_dict(expected_state.data)
		print("OLD STATE")
		print_dict(STATE.data)
		expected_effects = get_expected_effects(STATE.data, expected_state.data)


		# Publish task_network_info
		tn = TaskNetwork()
		tn.task_name = task_name
		tn.operator = op_str
		tn.effect = expected_effects
		TN_INFO_PUBLISHER.publish(tn)

		# Get response from failure_notification_server
		print("waiting for failure notification server")
		rospy.wait_for_service('failure_notification_server')
		print("failure notification server running")
		failure_notifier = rospy.ServiceProxy('failure_notification_server', FailureNotification)
		success = failure_notifier(json.dumps(expected_effects), json.dumps(STATE.data))
		print success

		if success == "True":
			continue
		else:
			# Request resolution steps from failure_resolution_server
			rospy.wait_for_service('failure_resolution_service')
			failure_resolver = rospy.ServiceProxy('failure_resolution_service', FailureResolution)
			resolution = failure_resolver(op_str, json.dumps(STATE.data))
			# If resolution is None:
			if resolution.resolution_action=="None":
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
			resolution_data = resolution_data[resolution_action_name]
			params = resolution_data['parameterization']
			params.insert(0, str(resolution_action_name))
			task = tuple(params)
			subplan = pyhop.pyhop(STATE, [task], verbose=3)
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
	state_dict = json.loads(msg.data)
	STATE = pyhop.State('STATE')
	STATE.data = state_dict

def command_in(msg):
	# Parse command and parameters.
	#TODO parse command and parameters
	'''
	task_name = msg.keys()[0]
	params = msg["fetch"]
	params.insert(0, str(task_name))
	task = tuple(params)
	'''
	task_name = "dummy"
	task = [('move_to', 'blue_5m_room')]
	# Run PyHOP to get a plan (i.e. list of sequenced primitive operators to execute)
	#print STATE.data
	#print "pre state"
	plan = pyhop.pyhop(STATE, task, verbose=2)
	# Execute the plaz
	if plan is False:
		print "Error:  Plan not found."
	else:
		execute_plan(task_name, plan)

def init_listeners():
	rospy.Subscriber("/dsi/world_state", String, world_state_in)

def create_publishers():
	global TN_INFO_PUBLISHER
	global RESOLUTION_REQUEST_PUBLISHER
	global RESOLUTION_DB_UPDATE_PUBLISHER
	# TODO: Need to make sure the message names, data types, etc. are correct
	TN_INFO_PUBLISHER = rospy.Publisher('task_network_info', TaskNetwork, queue_size=1)
	RESOLUTION_REQUEST_PUBLISHER = rospy.Publisher('/dsi/gui_startup', String, queue_size=1)
	RESOLUTION_DB_UPDATE_PUBLISHER = rospy.Publisher('/dsi/resolution_actions', String, queue_size=1)

def get_expected_effects(world_state, expected_state):
	expected_effects = {}
	for obj in expected_state:
		expected_effects[obj] = {}
		for attr in world_state[obj]:
			if type(attr) is list:
				if sorted(world_state[obj][attr])!=sorted(expected_state[obj][attr]):
					expected_effects[obj][attr] = expected_state[obj][attr]
			else:
				if world_state[obj][attr] != expected_state[obj][attr]:
					expected_effects[obj][attr] = expected_state[obj][attr]
		if not expected_effects[obj]:
			expected_effects.pop(obj)
	return expected_effects

def print_dict(dict):
	for obj in dict:
		print(obj)
		for item in dict[obj]:
			print("{} : {}".format(item, dict[obj][item]))

def main():
	global queue_lock

	rospy.init_node("planner_executor")
	''' initialize service action '''
	rospy.wait_for_service('/dsi/action_server')
	global action_server
	action_server = rospy.ServiceProxy('/dsi/action_server', action_service)
	init_listeners()
	create_publishers()
	pyhop.declare_operators(pd.move_to, pd.open_door, pd.detect, pd.pickup, pd.set_down, pd.unlock_door, pd.turn_on_lights)
	pyhop.declare_methods('navigate_to', pd.navigate_to)
	pyhop.declare_methods('fetch', pd.fetch)
	raw_input("Press Enter to continue...")
	command_in(INITIAL_COMMAND)
	rospy.spin()



if __name__ == '__main__':
	main()
