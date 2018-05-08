#! /usr/bin/env python2

import rospy
import rosparam
import json
from world_state.read_gazebo import readGazebo
from world_state.abstraction_engine import AbstractionEngine
from dsi_world_state.srv import world_state


#TODO works for now, don't like defining service in node
def world_state_handler(req):
    return json.dumps(world_space_dict)

if __name__ == '__main__':
    rospy.init_node('world_state')
    config_loc = rospy.get_param('/dsi/world_state/config_file_location')
    print config_loc
    # TODO havent decided if I want to share world space dictionary with other
    # things yet
    world_space_dict = {}
    gazebo_reader = readGazebo(ws_dict=world_space_dict, json_path=config_loc )
    rospy.sleep(1)
    abstraction_engine = AbstractionEngine(world_space_dict, config_loc)

    ''' world state server works for now'''
    s = rospy.Service('/dsi/world_state_server', world_state, world_state_handler)



    try:
        rospy.spin()
    except:
        rospy.logerror("mister meeseeks is finally free")
