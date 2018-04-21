#! /usr/bin/env python2

import rospy



import sys
import tf
import json
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist


class readGazebo(object):

    ''' class to read objects from gazebo and place in world state dictionary '''
    def __init__(self, ws_dict, freq=10):
        self.count = 1000/freq
        self.counter = 0
        self.dict = ws_dict
        #TODO convert predicate dictionary to JSON and make it loadable
        self.predicate_dict = {"cafe_beer" : "non_static",
                               "pringles": "non_static",
                               "test_zone": "building",
                               "folding_table_4x2": "static",
                               "movo": "robot"}

    def parser(self, data):
        if self.counter < self.count:
            self.counter += 1
            return

        print data.name
        self.counter = 0
        for i, obj_name in enumerate(data.name):
            log_name = self._dict_checker(obj_name)
            if log_name is not None:
                self._dict_new_obj(obj_name)

            self.dict[obj_name] = {"location": self._pose_to_list(data.pose[i]),
                                   "velocity": self._twist_to_list(data.twist[i])}
                                   #TODO move _name to type to _dict_checker
            sys.stdout.write(obj_name+" : ")
            #print self.dict[obj_name]["type"]
        print


    def _dict_new_obj(self, obj_name):
        ''' initializes new object within the world state dictionary'''

        #self.dict[obj_name] = {""}
        pass


    def _dict_checker(self, obj_name):
        '''checks if object is in dictionary and creates ros info for new '''
        try:
            self.dict[obj_name]
        except:
            rospy.loginfo("new object {} created".format(obj_name))
            return obj_name


    def _dict_remove(self, ):
        ''' removes objects that are no longer being published '''
        pass

    def _name_to_type(self, name):
        ''' takes object name and gives it a type '''
        #TODO unhappy with this method
        for key in  self.predicate_dict:
            ''' check if type exists'''
            if key in name:
                print "found"

        pass

    def _pose_to_list(self, pose):
        ''' take in a ros pose message and convert to list '''
        data = []
        data.append(pose.position.x)
        data.append(pose.position.y)
        data.append(pose.position.z)
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        ''' Euler angles in radians '''
        data.append(euler[0]) #Roll
        data.append(euler[1]) #Pitch
        data.append(euler[2]) #Yaw
        return data

    def _twist_to_list(self, twist):
        ''' take in twist message and convert to list linear then angular'''
        data = []
        data.append(twist.linear.x)
        data.append(twist.linear.y)
        data.append(twist.linear.z)
        data.append(twist.angular.x)
        data.append(twist.angular.y)
        data.append(twist.angular.z)
        return data

    def _predicate_adder(self, obj):
        ''' adds predicates based on object type '''
        pass

    def _non_static_pred(self):
        ''' adds predicates for non static objects '''
        pass

    def _static_pred(self):
        ''' adds predicates for static objects '''

    def _building_pred(self):
        ''' adds predicates for buildings '''




if __name__ == '__main__':
    rospy.init_node('World_Reader')
    world_space_dict = {}
    gazebo_reader = readGazebo(ws_dict=world_space_dict)
    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_reader.parser)

    try:
        rospy.spin()
    except:
        rospy.logerror("mister meeseeks is finally free")
