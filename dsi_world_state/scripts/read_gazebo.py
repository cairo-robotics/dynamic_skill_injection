#! /usr/bin/env python2

import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist


class readGazebo(object):

    predicate_dict = {"cafe_beer" : "non_static",
                      "pringles": "non_static",
                      "test_zone": "building",
                      "folding_table_4x2": "static" }

    ''' class to read objects from gazebo and place in world state dictionary '''
    def __init__(self, ws_dict, freq=10):
        self.count = 1000/freq
        self.counter = 0
        self.dict = ws_dict

    def parser(self, data):
        if self.counter < self.count:
            self.counter += 1
            return

        print data.name
        self.counter = 0
        for i, obj in enumerate(data.name):
            self.dict[obj] = {"location": _pose_to_tuple(data.pose[i]),
                              "velocity": _twist_to_tuple(data.twist[i]),
                              "type":}


    def _dict_checker(self, name):
        '''checks if object is in dictionary and creates ros info for new '''
        pass

    def _dict_remove(self):
        ''' removes objects that are no longer being published '''
        pass

    def _name_to_type(self, name):
        ''' takes object name and gives it a type '''
        pass

    def _pose_to_tuple(self, pose):
        ''' take in a ros pose message and convert to tuple '''
        pass

    def _twist_to_tuple(self, twist):
        ''' take in twist message and convert to tuple '''
        pass

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
        pass
