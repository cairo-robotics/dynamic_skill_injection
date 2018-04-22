#! /usr/bin/env python2

import rospy
import rosparam
from world_state.read_gazebo import readGazebo


if __name__ == '__main__':
    rospy.init_node('World_Reader')
    config_loc = rospy.get_param('/dsi/world_state/config_file_location')
    print config_loc
    #TODO havent decided if I want to share world space dictionary with other things yet
    world_space_dict = {}
    gazebo_reader = readGazebo(ws_dict=world_space_dict, json_path=config_loc )

    try:
        rospy.spin()
    except:
        rospy.logerror("mister meeseeks is finally free")
