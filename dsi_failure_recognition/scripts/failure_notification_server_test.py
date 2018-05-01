#!/usr/bin/env python

import rospy
import json
from dynamic_skill_injection_msgs.srv import FailureNotification

# Not strictly required to have all data types, only those that are affected by the current task operator.
effect = {
    "mug1": {
        "type": "container",
        "location_xyz": [0, 0, 0, 0, 0, 0],
        "location_str": [["on", "table1"], ["in", "kitchen1"]],
        "size": [5.0, 5.0, 5.0],
        # Goal is to have detected go from False -> True
        "detected": True,
        "open": True,
        "empty": True
    }
}

# Not strictly required to have all data types, only those that are affected by the current task operator.
world_state = {
    "mug1": {
        "type": "container",
        "location_xyz": [0, 0, 0, 0, 0, 0],
        "location_str": [["in", "kitchen1"], ["on", "table1"]],
        "size": [5.0, 5.0, 5.0],
        # Goal is to have detected go from False -> True
        "detected": False,
        "lights_on": False,
        "open": True,
        "empty": True
    }
}


def failure_notification_client(effect, world_state):
    rospy.wait_for_service('failure_notification_server')
    try:
        rospy.init_node('failure_notification_client')
        failure_notifier = rospy.ServiceProxy('failure_notification_server', FailureNotification)
        response = failure_notifier(effect, world_state)
        return response
    except rospy.ServiceException as e:
        print "Service call failed: {}".format(e)


if __name__ == "__main__":
    try:
        effect = json.dumps(effect)
        world_state = json.dumps(world_state)
        response = failure_notification_client(effect, world_state)
        rospy.loginfo(response)
    except rospy.ROSInterruptException:
        pass