#!/usr/bin/python

import rospy
import json
from functools import partial
from failure_resolution.resolution import FailureMapper
from std_msgs.msg import String
from dynamic_skill_injection_msgs.srv import FailureResolution


def detect_failure(req, mapper):
    return mapper.request_handler(req)


def failure_resolution_server(callback):
    rospy.init_node('failure_resolution_server')
    s = rospy.Service('failure_resolution_service', FailureResolution, callback)
    rospy.spin()


if __name__ == "__main__":
    mapper = FailureMapper()
    print(mapper)
    try:
        rospy.Subscriber('dsi/resolution_actions', String,  mapper.update_action_resolution_callback)
        callback = partial(detect_failure, mapper=mapper)
        failure_resolution_server(callback)
    except rospy.ROSInterruptException:
        pass
