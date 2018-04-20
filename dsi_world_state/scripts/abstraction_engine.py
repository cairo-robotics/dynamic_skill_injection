#!/usr/bin/env python

'''
Functions for turning numerical representations of objects into predicate statements
'''

def generalize_batch(obs_state):
	for obj1 in obs_state:
		obj1_loc_xyz = obs_state[obj1]['location_xyz']
		obj1_loc_str = obs_state[obj1]['location_str']
		obj1_dims = obs_state[obj1]['size']
		for obj2 in obs_state:
			obj2_loc_xyz = obs_state[obj2]['location_xyz']
			obj2_dims = obs_state[obj2]['size']
			if obj1 is not obj2:
				generalize_single(obj1_loc_str, obj1_loc_xyz, obj1_dims, obj2, obj2_loc_xyz, obj2_dims)

def generalize_single(obj1_loc_str, obj1_loc_xyz, obj1_dims, obj2, obj2_loc_xyz, obj2_dims):
	relationship = get_relationship(obj1_loc_xyz, obj1_dims, obj2_loc_xyz, obj2_dims)
	if relationship and relationship[0] and relationship not in obj1_loc_str:
		obj1_loc_str.append((relationship, obj2))

def get_relationship(obj1_loc, obj1_dims, obj2_loc, obj2_dims):
	relationship = None
	relationship = is_in(obj1_loc, obj1_dims, obj2_loc, obj2_dims)
	if not relationship:
		relationship = is_on(obj1_loc, obj1_dims, obj2_loc, obj2_dims)
	return relationship

def is_in(obj1_loc, obj1_dims, obj2_loc, obj2_dims):
	result = None
	obj1_xmin, obj1_xmax, obj1_ymin, obj1_ymax, obj1_zmin, obj1_zmax = get_corners(obj1_loc, obj1_dims)
	obj2_xmin, obj2_xmax, obj2_ymin, obj2_ymax, obj2_zmin, obj2_zmax = get_corners(obj2_loc, obj2_dims)
	if obj1_xmin>=obj2_xmin and obj1_xmax<=obj2_xmax:
		if obj1_ymin>=obj2_ymin and obj1_ymax<=obj2_ymax:
			if obj1_zmin>=obj2_zmin and obj1_zmax<=obj2_zmax:
				result = 'in'
	return result

def is_on(obj1_loc, obj1_dims, obj2_loc, obj2_dims):
	VERT_MEASUREMENT_TOLERANCE = 0.05
	result = None
	obj1_x = obj1_loc[0]
	obj1_y = obj1_loc[1]
	obj1_zmin = obj1_loc[2] - (.5 * obj1_dims[2])
	obj2_xmin, obj2_xmax, obj2_ymin, obj2_ymax, obj2_zmin, obj2_zmax = get_corners(obj2_loc, obj2_dims)
	if obj1_x>=obj2_xmin and obj1_x<=obj2_xmax:
		if obj1_y>=obj2_ymin and obj1_y<=obj2_ymax:
			if obj1_zmin>=obj2_zmax-VERT_MEASUREMENT_TOLERANCE and obj1_zmin<=obj2_zmax+VERT_MEASUREMENT_TOLERANCE:
				result = 'on'
	return result

def get_corners(obj_loc, obj_dims):
	x_min = obj_loc[0] - (.5 * obj_dims[0])
	x_max = obj_loc[0] + (.5 * obj_dims[0])
	y_min = obj_loc[1] - (.5 * obj_dims[1])
	y_max = obj_loc[1] + (.5 * obj_dims[1])
	z_min = obj_loc[2] - (.5 * obj_dims[2])
	z_max = obj_loc[2] + (.5 * obj_dims[2])
	return x_min, x_max, y_min, y_max, z_min, z_max
