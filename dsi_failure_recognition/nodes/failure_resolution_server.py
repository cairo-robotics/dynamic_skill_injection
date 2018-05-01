#!/usr/bin/python

import rospy
import json
from functools import partial
from failure_resolution.resolution import FailureMapper
from dynamic_skill_injection_msgs.srv import TaskFailure


resolution_actions = {
    "search": [
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
        ]
    ]
}

def detect_failure(mapper, req)
    return detect.check_failure(req)

def failure_resolution_server(callback):
    rospy.init_node('failure_resolution_server')
    s = rospy.Service('failure_resolution_server', TaskFailure, callback)
    rospy.spin()

if __name__ == "__main__":
    mapper = FailureMapper()
    try:
        rospy.Subscriber('dsi/resolution_actions', String,  detector.action_resolution_callback)
        callback = partial(detect_failure, mapper = mapper)
        failure_notification_server(callback)
    except rospy.ROSInterruptException:
        pass