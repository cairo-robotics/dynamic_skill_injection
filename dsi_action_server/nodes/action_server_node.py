#! /usr/bin/env python2

import rospy
import json
import rospkg
import sys

from action_server.action_server import actionServer
from std_msgs.msg import String
from geometry_msgs.msg import Pose



if __name__ == '__main__':

    rospy.init_node("the_action_server")

    rospack = rospkg.RosPack()
    base_path = rospack.get_path("dsi_action_server")
    actions_loc = base_path + "/config/actions.json"

    the_server = actionServer(configs="configs.json", methods="methodlib.json")

    try:
         rospy.spin()
    except Exception as e:
        rospy.logerr("action server spin kill: {}".format(e))
