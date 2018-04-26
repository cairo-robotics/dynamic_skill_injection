#! /usr/bin/env python2

import rospy
import tf

from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String


class holdMovoUpright(object):

    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

    def service_pub(self):

        model_state = self.get_model_state( 'movo', '')

        quaternion = (
        model_state.pose.orientation.x,
        model_state.pose.orientation.y,
        model_state.pose.orientation.z,
        model_state.pose.orientation.w)
        euler_t = tf.transformations.euler_from_quaternion(quaternion)
        euler = [0]*3
        euler[2] = euler_t[2]
        quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

        new_model_state = ModelState()
        new_model_state.model_name = 'movo'
        new_model_state.pose.orientation.x = quaternion[0]
        new_model_state.pose.orientation.y = quaternion[1]
        new_model_state.pose.orientation.z = quaternion[2]
        new_model_state.pose.orientation.w = quaternion[3]
        new_model_state.pose.position = model_state.pose.position
        new_model_state.pose.position.z = 0
        new_model_state.twist = model_state.twist

        self.set_model_state(new_model_state)
        

if __name__ == '__main__':
    movo_upright = holdMovoUpright()
    while not rospy.is_shutdown():
        movo_upright.service_pub()
        rospy.sleep(.2)
