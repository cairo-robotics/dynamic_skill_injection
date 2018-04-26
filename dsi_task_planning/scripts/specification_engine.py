#!/usr/bin/env python

'''
Functions for turning numerical representations of objects into predicate statements
'''


from ast import literal_eval


def specify(state, preposition, obj, tgt):
	robot_loc = state['robot1']['location_xyz']
	obj_dims = state[obj]['size']
	tgt_type = state[tgt]['type']
	tgt_loc = state[tgt]['location_xyz']
	tgt_dims = state[tgt]['size']
	# TODO: These calculations assume object pose is grid-aligned with axes (i.e. no rotation)
	if preposition=='on':
		result = specify_on(robot_loc, obj_dims, tgt_loc, tgt_dims)
	elif preposition=='to':
		result = specify_to(robot_loc, obj_dims, tgt_type, tgt_loc, tgt_dims)
	return result

def specify_on(robot_loc, obj_dims, tgt_loc, tgt_dims):
	# TODO: Make position for placing object more elegant instead of just in the middle of tgt loc
	result = (tgt_loc[0], tgt_loc[1], tgt_loc[2] + (.5 * tgt_dims[2]))
	return result

def specify_to(robot_loc, obj_dims, tgt_type, tgt_loc, tgt_dims):
	# TODO: Need to fix the result so it accounts for robot location
	if tgt_type=='room':
		# Goes to center of room
		result = (tgt_loc[0], tgt_loc[1], robot_loc[2])
	else:
		# FIXME:  This result is broken need to create better approach given our scene geometry
		result = (tgt_loc[0], tgt_loc[1] - tgt_dims[1], robot_loc[2])
	return result
