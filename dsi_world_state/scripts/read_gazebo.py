#! /usr/bin/env python2

import rospy



import sys
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist


class readGazebo(object):

    #TODO convert predicate dictionary to JSON and make it loadable
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
            log_name = self._dict_checker(obj)
            self.dict[obj] = {"location": self._pose_to_tuple(data.pose[i]),
                              "velocity": self._twist_to_tuple(data.twist[i]),
                              "type": "test"}
            sys.stdout.write(obj+" : ")
            print self.dict[obj]["location"]


        print


    def _dict_checker(self, name):
        '''checks if object is in dictionary and creates ros info for new '''
        try:
            self.dict[name]
        except:
            rospy.loginfo("new object {} created".format(name))
            return name


    def _dict_remove(self):
        ''' removes objects that are no longer being published '''
        pass

    def _name_to_type(self, name):
        ''' takes object name and gives it a type '''
        pass

    def _pose_to_tuple(self, pose):
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

    def _twist_to_tuple(self, twist):
        ''' take in twist message and convert to tuple '''
        return 1

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
