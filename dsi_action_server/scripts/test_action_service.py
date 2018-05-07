#! /usr/bin/env python2


import rospy
import json

from dsi_action_server.srv import action_service


if __name__ == '__main__':
    rospy.init_node("test_node")
    action_server_service = rospy.ServiceProxy('/dsi/action_server',action_service)

    send_string = json.dumps({"method":"push_door"})

    print action_server_service(send_string)
    print send_string
