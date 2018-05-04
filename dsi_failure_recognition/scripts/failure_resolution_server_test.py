#!/usr/bin/env python

import rospy
import json
from dynamic_skill_injection_msgs.srv import FailureResolution

# Not strictly required to have all data types, only those that are affected by the current task operator.
world_state = {
    "mug1": {
        "type": "container",
        "location_xyz": [0, 0, 0, 0, 0, 0],
        "location_str": [["in", "kitchen1"], ["on", "table1"]],
        "size": [5.0, 5.0, 5.0],
        # Goal is to have detected go from False -> True
        "detected": False,
        "open": True,
        "empty": True
    },
    "lights_on": False,
}

operator = "search"


def failure_notification_client(operator, world_state):
    rospy.wait_for_service('failure_resolution_server')
    try:
        rospy.init_node('failure_resolution_client')
        failure_notifier = rospy.ServiceProxy('failure_resolution_server', FailureResolution)
        response = failure_notifier(operator, world_state)
        return response
    except rospy.ServiceException as e:
        print "Service call failed: {}".format(e)


if __name__ == "__main__":
    try:
        world_state = json.dumps(world_state)
        response = failure_notification_client(operator, world_state)
        rospy.loginfo(response)
    except rospy.ROSInterruptException:
        pass