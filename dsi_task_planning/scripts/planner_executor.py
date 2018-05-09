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
import pdb

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
INITIAL_COMMAND = {'grab_beer':['']}



def execute_plan(task_name, plan):
	# plan = [('op1', 'arg1', 'arg2'), ('op2', 'arg1', 'arg2')]

	execution_success = True
	for idx,  op_and_params in enumerate(plan):
		op_str = op_and_params[0]
		operator = getattr(pd, op_and_params[0])
		op_args = op_and_params[1:]

		# pdb.set_trace()

		expected_state = copy.deepcopy(STATE)
		initial_state = copy.deepcopy(expected_state)
		operator(expected_state, *op_args)



		# Publish task_command for execution
		# TODO: publish the command
		action_server_command(op_and_params)

		# Determine expected changes to world state as a result of primitive
		expected_effects = get_expected_effects(initial_state.data, expected_state.data)




		# Publish task_network_info
		tn = TaskNetwork()
		tn.task_name = task_name
		tn.operator = op_str
		tn.effect = expected_effects
		TN_INFO_PUBLISHER.publish(tn)

		rospy.sleep(1.1)
		# Get response from failure_notification_server
		print("waiting for failure notification server")
		rospy.wait_for_service('failure_notification_server')
		print("failure notification server running")
		failure_notifier = rospy.ServiceProxy('failure_notification_server', FailureNotification)
		success = failure_notifier(json.dumps(expected_effects), json.dumps(STATE.data))

		resolution_update_data = None
		while success.success == False:
			rospy.loginfo("failure in step {}".format(op_and_params))
			# Request resolution steps from failure_resolution_server
			rospy.wait_for_service('failure_resolution_service')
			failure_resolver = rospy.ServiceProxy('failure_resolution_service', FailureResolution)
			resolution_response = failure_resolver(op_str, json.dumps(STATE.data))
			resolution_action = json.loads(resolution_response.resolution_action)
			if resolution_action == {}:
				resolution_action = None

			# pdb.set_trace()
			# If resolution is None:
			if resolution_action == None:
				# Request recommendation from user
				RESOLUTION_REQUEST_PUBLISHER.publish('run')
				human_command_msg = rospy.wait_for_message("/dsi/human_command", String)
				print(human_command_msg.data)
				resolution_action = json.loads(human_command_msg.data)
				if resolution_action == None:
					return False
				resolution_update_data = {op_str: resolution_action}

			# Parse resolution into input for PyHOP and get plan, called 'action'
			resolution_action_name = resolution_action.keys()[0]
			resolution_data = resolution_action[resolution_action_name]
			params = resolution_data['parameterization']
			params.insert(0, str(resolution_action_name))
			resolution_ops_and_params = tuple(params)
			# subplan = pyhop.pyhop(STATE, [resolution_ops_and_params], verbose=3)
			# # Execute resolution
			# if subplan is False:
			# 	print "Error:  Subplan not found."
			# 	return False
			# else:
			# 	execute_plan('Failure Resolution: ' + op_str, subplan)

			# Execute resolution action.
			action_server_command(resolution_ops_and_params)

			rospy.sleep(1.1)
			# Retry failed operator.  Publish task_command for execution
			expected_state = copy.deepcopy(STATE)
			initial_state = copy.deepcopy(expected_state)
			result = operator(expected_state, *op_args)
			# Determine expected changes to world state as a result of primitive
			expected_effects = get_expected_effects(initial_state.data, expected_state.data)
			if result == False:
				continue
			# Publish task_command for execution
			# TODO: publish the command
			action_server_command(op_and_params)




			# Publish task_network_info
			tn.task_name = task_name
			tn.operator = op_str
			tn.effect = expected_effects
			TN_INFO_PUBLISHER.publish(tn)
			rospy.sleep(1.1)
			# Get response from failure_notification_server
			success = failure_notifier(json.dumps(expected_effects), json.dumps(STATE.data))
		rospy.loginfo("Action success")
		if resolution_update_data is not None:
			rospy.loginfo("Storing resolution action")
			# Publish message to update resolution database
			RESOLUTION_DB_UPDATE_PUBLISHER.publish(json.dumps(resolution_update_data))

	return True

def world_state_in(msg):
	global STATE
	state_dict = json.loads(msg.data)
	STATE = pyhop.State('STATE')
	STATE.data = state_dict

def action_server_command(op_and_params):
	if op_and_params[0] == "move_to":
		rospy.loginfo("sent move to command")
		if op_and_params[1] == "red_5m_room":
			response = action_server(json.dumps({"method":"teleport_red"}))

		elif op_and_params[1] == "blue_5m_room":
			response = action_server(json.dumps({"method":"teleport_blue"}))

		elif op_and_params[1] == "table":
			response = action_server(json.dumps({"method":"teleport_table"}))

		else:
			rospy.logwarn("move_to not found")

	elif  op_and_params[0] == "pickup":
		rospy.loginfo("sent pickup command")
		response = action_server(json.dumps({"method":"grab_beer"}))
		rospy.loginfo("received {} response".format(response))
	elif op_and_params[0] == "set_down":
		rospy.loginfo("create setdown command")
	elif op_and_params[0] == "turn_on_lights":
		response = action_server(json.dumps({"method":"light_on_spot_0"}))
		rospy.loginfo("light turned on")
	elif op_and_params[0] == "turn_off_lights":
		response = action_server(json.dumps({"method":"light_off_spot_0"}))
		rospy.loginfo("light turned off")
	else:
		rospy.logwarn("invalid action command operator")


def command_in(msg):
	# Parse command and parameters.
	#TODO parse command and parameters
	'''
	task_name = msg.keys()[0]
	params = msg["fetch"]
	params.insert(0, str(task_name))
	task = tuple(params)
	'''
	task_name = "fetch"
	#task = [('fetch', 'blue_5m_room')]
	task = [('move_to', 'red_5m_room'),
			('detect', 'cafe_beer'),
			 ('pickup', 'cafe_beer')]
	# Run PyHOP to get a plan (i.e. list of sequenced primitive operators to execute)
	#print STATE.data
	#print "pre state"
	#action_server_command(('turn_on_lights', 'red_5m_room'))
	#rospy.sleep(1.1)
	#plan = pyhop.pyhop(STATE, task, verbose=2)
	#rospy.sleep(1.1)
	action_server_command(('turn_off_lights', 'red_5m_room'))
	# Execute the plaz
	#if plan is False:
	#	print "Error:  Plan not found."
	#else:
	execute_plan(task_name, task)

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
	not_checked = ["location_xyz", "orientation_rpq", "velocity"]
	expected_effects = {}
	for obj in expected_state:
		expected_effects[obj] = {}
		for attr in world_state[obj]:
			if attr in not_checked:
				continue
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
	pyhop.declare_operators(pd.move_to, pd.pickup, pd.set_down)
	#pyhop.declare_methods('navigate_to', pd.navigate_to)
	#pyhop.declare_methods('fetch', pd.fetch)
	raw_input("Press Enter to continue...")
	command_in(INITIAL_COMMAND)
	print("finished")



if __name__ == '__main__':
	main()
