#! /usr/bin/env python2

import rospy
import json
from dsi_world_state.srv import world_state


def print_dict(dict):
    for obj in dict:
        print ""
        print obj
        for item in dict[obj]:
            print "{} : {}".format(item, dict[obj][item])

def main():
    rospy.wait_for_service('/dsi/world_state_server')
    world_state_server = rospy.ServiceProxy('/dsi/world_state_server', world_state)
    while(not rospy.is_shutdown()):
        test = raw_input("press enter to poll world state")
        print_dict(json.loads(world_state_server('blank').world_state))

if __name__ == '__main__':
    main()
