#! /usr/bin/env python2

import rospy
import tf
import json
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String

class readGazebo(object):
    ''' class to read objects from gazebo and place in world state dictionary '''
    #TODO use parameter server for json_path and freq
    def __init__(self, ws_dict, freq=10, json_path="/home/jeff/movo_ws/src/dynamic_skill_injection/dsi_world_state/config/config1.json"):
        """
        initialize the gazebo reading class object

        Parameters
        ----------
        ws_dict: world space dictionary
        freq: frequency at which dictionary will be updated and published
        json_path: The location of the json configuration file
        """
        self.hz_count = 1000/freq
        self.counter = 0
        self.dict = ws_dict
        configs = json.load(open(json_path))
        self.obj_types = configs['obj_types']
        self.type_primitives = configs['type_primitives']
        self.obj_primitives = configs['obj_primitives']
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.parser)
        self.pub_ws = rospy.Publisher("/dsi/world_state", String, queue_size=1)

    def parser(self, data):
        """
        takes gazebo data and places into dictionary to be sent out on then
        world state topic TODO

        Parameters
        ---------
        data: the data from the topic "/gazebo/model_states"
        """
        if self.counter < self.hz_count:
            self.counter += 1
            return

        self.counter = 0
        self._dict_purge(data.name)
        for i, obj_name in enumerate(data.name):
            self._dict_checker(obj_name)
            pose_array = self._pose_to_list(data.pose[i])
            self.dict[obj_name]["location_xyz"] = pose_array[0:3]
            self.dict[obj_name]["orientation_rpq"] = pose_array[3:6]
            self.dict[obj_name]["velocity"] = self._twist_to_list(data.twist[i])
            #Just dump whole dict together?
            rospy.logdebug(obj_name + " : " + json.dumps(self.dict[obj_name]))
        self.pub_ws.publish(json.dumps(self.dict))

    def _dict_new_obj(self, obj_name):
        """
        Initializes new object within the world state dictionary

        Parameters
        ----------
        obj_name: name of object from the gazebo world
        """
        try:
            obj_type, obj_base = self._name_to_type(obj_name)
            self.dict[obj_name] = self.type_primitives[obj_type]
            for key in self.obj_primitives[obj_name]:
                self.dict[obj_name][key] = self.obj_primitives[obj_base][key]
                print self.dict[obj_name]

        except:
            rospy.logerr("error create object: {}".format(obj_name))

    def _dict_checker(self, obj_name):
        """
        checks if object is in dictionary and calls _dict_new_obj to created
        objects that don't yet exist in dictinary.

        Parameters
        ----------
        obj_name: name of object from the gazebo world
        """
        try:
            self.dict[obj_name]
        except:
            self._dict_new_obj(obj_name)
            rospy.loginfo("new object {} created".format(obj_name))

    def _dict_purge(self, obj_names):
        """
        removes objects that are no longer being published
        publishes info message for the object

        Parameters
        ----------
        obj_names: list of all objects currenly in gazebo
        """
        for name in self.dict:
            if name in obj_names:
                pass
            else:
                del self.dict[name]
                rospy.loginfo("{} was removed from dictionary".format(name))
                # cannot change dictionart size durring iteration
                # will catch multiple removes succesively
                break

    def _name_to_type(self, obj_name):
        """
        takes object name and gives it a type if no tyype exists return
        none

        Parameters
        ----------
        obj_name: name of object from the gazebo world

        Returns
        -------
        obj_type: the type of object
        obj_base: the object base name
        """
        #TODO unhappy with this method error with beer vs cafe_beer
        for obj_base in self.obj_types:
            if obj_base in obj_name:
                obj_type = self.obj_types[obj_base]
                return obj_type, obj_base
        rospy.logwarn("object {} does not exist".format(obj_name))
        return "none"

    def _pose_to_list(self, pose):
        """
        Takes in a ros Pose message and converts to a list

        Parameters
        ----------
        pose: ros pose message

        Returns
        -------
        list: list of pose data
        """
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
        """
        Takes in a ros Twist message and converts to a list

        Parameters
        ----------
        Twist: ros Twist message

        Returns
        -------
        list: list of twist data
        """
        data = []
        data.append(twist.linear.x)
        data.append(twist.linear.y)
        data.append(twist.linear.z)
        data.append(twist.angular.x)
        data.append(twist.angular.y)
        data.append(twist.angular.z)
        return data


if __name__ == '__main__':
    rospy.init_node('World_Reader')
    world_space_dict = {}
    gazebo_reader = readGazebo(ws_dict=world_space_dict)

    try:
        rospy.spin()
    except:
        rospy.logerror("mister meeseeks is finally free")
