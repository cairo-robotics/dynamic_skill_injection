#!/usr/bin/env python

import rospy
from failure_detection.task_failure import TaskFailureDetector
from dynamic_skill_injection_msgs.srv import FailureNotification


def detect_failure(req):
    detector = TaskFailureDetector()
    return detector.handle_request(req)


def failure_notification_server():
    rospy.init_node('failure_notification_server')
    s = rospy.Service('failure_notification_server', FailureNotification, detect_failure)
    rospy.loginfo("failure_notification_server up an running!")
    rospy.spin()


if __name__ == "__main__":
    try:
        failure_notification_server()
    except rospy.ROSInterruptException:
        pass
