#!/usr/bin/python

import rospy
import json
from failure_resolution import ResolutionLibrary
from std_msgs.msg import String


def generate_resolution_library():

    rospy.init_node('resolution_library_publisher', anonymous=False)

    library = ResolutionLibrary()

    # We pass the bound detector method human_command_callback as the callback  for each the 
    # /human_command subscrubers, which update the ResolutionLibrary instance's member variable.
    rospy.Subscriber('/human_command', String,  library.human_command_callback)

    pub = rospy.Publisher('dsi/resolution_actions', String, queue_size=10)

    rate = rospy.Rate(10)  # 10hz 
    while not rospy.is_shutdown():
        message = json.loads(library.resolution_actions)
        pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    try:
        generate_resolution_library()
    except rospy.ROSInterruptException:
        pass
