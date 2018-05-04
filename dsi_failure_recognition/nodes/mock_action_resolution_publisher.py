#!/usr/bin/python

import rospy
import json
from std_msgs.msg import String

resolution_actions = {
    "search": {
        "turn_on_lights": {
            "failure_conditions": {
                "lights_on": False
                }
            },
        "lift_object": {
            "failure_conditions": {
                "target_occluded": True
            }
        }
    }
}


def publish_resolution_actions():
    rospy.init_node('resolution_actions_publisher', anonymous=False)
    pub = rospy.Publisher('dsi/resolution_actions', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(json.dumps(resolution_actions))
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_resolution_actions()
    except rospy.ROSInterruptException:
        pass
