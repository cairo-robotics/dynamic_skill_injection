#! /usr/bin/env python2

import rospy
import json
import rospkg
import sys

from action_server.action_server import actionServer
from std_msgs.msg import String
from geometry_msgs.msg import Pose


def world_state_callback(data):
    global world_state
    world_state = json.loads(data.data)
    rospy.logdebug("received world state in action server")



if __name__ == '__main__':

    rospy.init_node("beta_action_server")

    rospy.Subscriber("/dsi/world_state", String, world_state_callback)

    rospack = rospkg.RosPack()
    base_path = rospack.get_path("dsi_action_server")
    actions_loc = base_path + "/config/actions.json"
    #actions =json.load(open(actions_loc))
    test = actionServer()
    rospy.sleep(2)
    test.lgripper.command(.01)
    rospy.sleep(1)
    test.lgripper.command(0.55)







    pass
