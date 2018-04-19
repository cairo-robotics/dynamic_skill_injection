#! /usr/bin/env python2

import rospy

from gazebo_msgs import ModelStates






def parser():
    pass


if __name__ == '__main__':

    rospy.init_node('World_Reader')

    try:
        rospy.spin()
    except:
        pass
