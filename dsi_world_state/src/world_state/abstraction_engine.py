#!/usr/bin/env python

import rospy
import json



'''
Functions for turning numerical representations of objects into predicate statements
'''


class abstractionEngine(object):
	def __init__(self, ws_dict, json_path, update_rate = 1,  vert_meas_tol = 0.05):
		"""
		initialize the abstraction engine with shared memory location of then
		world state dictionary

		Parameters
		----------
		ws_dict: world space dictionary
		update_rate = frequency of abstraction updates
		vert_meas_tol: measurement of vertical tolerance
		"""

		self.VERT_MEASUREMENT_TOLERANCE = vert_meas_tol
		self.ws_dict = ws_dict
		configs = json.load(open(json_path))
		rospy.Timer(rospy.Duration(1.0/update_rate), self.callback)
		self.abstract_config = configs["abstract_list"]

	def callback(self, event):
		"""
		pulls out objects to be compared in abstraction engine
		"""
		obj_abst_array = []
		for obj in self.ws_dict:
			if self.ws_dict[obj]["type"] in self.abstract_config:
				obj_abst_array.append(obj)


		print obj_abst_array
		self.generalize_batch(obj_abst_array)



	def generalize_batch(self, obj_list):
		for obj1 in obj_list:
			obj1_loc_xyz = self.ws_dict[obj1]['location_xyz']
			obj1_loc_str = self.ws_dict[obj1]['location_str']
			obj1_dims = self.ws_dict[obj1]['dimensions']
			for obj2 in obj_list:
				obj2_loc_xyz = self.ws_dict[obj2]['location_xyz']
				obj2_dims = self.ws_dict[obj2]['dimensions']
				if obj1 is not obj2:
					self.generalize_single(obj1_loc_str, obj1_loc_xyz, obj1_dims, obj2, obj2_loc_xyz, obj2_dims)

	def generalize_single(self, obj1_loc_str, obj1_loc_xyz, obj1_dims, obj2, obj2_loc_xyz, obj2_dims):
		relationship = self.get_relationship(obj1_loc_xyz, obj1_dims, obj2_loc_xyz, obj2_dims)
		if relationship and relationship[0] and relationship not in obj1_loc_str:
			obj1_loc_str.append((relationship, obj2))
		print obj1_loc_str


	def get_relationship(self, obj1_loc, obj1_dims, obj2_loc, obj2_dims):
		relationship = None
		relationship = self.is_in(obj1_loc, obj1_dims, obj2_loc, obj2_dims)
		if not relationship:
			relationship = self.is_on(obj1_loc, obj1_dims, obj2_loc, obj2_dims)
		return relationship

	def is_in(self,obj1_loc, obj1_dims, obj2_loc, obj2_dims):
		result = None
		obj1_xmin, obj1_xmax, obj1_ymin, obj1_ymax, obj1_zmin, obj1_zmax = self.get_corners(obj1_loc, obj1_dims)
		obj2_xmin, obj2_xmax, obj2_ymin, obj2_ymax, obj2_zmin, obj2_zmax = self.get_corners(obj2_loc, obj2_dims)
		if obj1_xmin>=obj2_xmin and obj1_xmax<=obj2_xmax:
			if obj1_ymin>=obj2_ymin and obj1_ymax<=obj2_ymax:
				if obj1_zmin>=obj2_zmin and obj1_zmax<=obj2_zmax:
					result = 'in'
		return result

	def is_on(self, obj1_loc, obj1_dims, obj2_loc, obj2_dims):
		VERT_MEASUREMENT_TOLERANCE = self.VERT_MEASUREMENT_TOLERANCE
		result = None
		obj1_x = obj1_loc[0]
		obj1_y = obj1_loc[1]
		obj1_zmin = obj1_loc[2] - (.5 * obj1_dims[2])
		obj2_xmin, obj2_xmax, obj2_ymin, obj2_ymax, obj2_zmin, obj2_zmax = self.get_corners(obj2_loc, obj2_dims)
		if obj1_x>=obj2_xmin and obj1_x<=obj2_xmax:
			if obj1_y>=obj2_ymin and obj1_y<=obj2_ymax:
				if obj1_zmin>=obj2_zmax-VERT_MEASUREMENT_TOLERANCE and obj1_zmin<=obj2_zmax+VERT_MEASUREMENT_TOLERANCE:
					result = 'on'
		return result

	def get_corners(self, obj_loc, obj_dims):
		x_min = obj_loc[0] - (.5 * obj_dims[0])
		x_max = obj_loc[0] + (.5 * obj_dims[0])
		y_min = obj_loc[1] - (.5 * obj_dims[1])
		y_max = obj_loc[1] + (.5 * obj_dims[1])
		z_min = obj_loc[2] - (.5 * obj_dims[2])
		z_max = obj_loc[2] + (.5 * obj_dims[2])
		return x_min, x_max, y_min, y_max, z_min, z_max
