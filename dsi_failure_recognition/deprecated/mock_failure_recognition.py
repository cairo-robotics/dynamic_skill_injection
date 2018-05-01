#!/usr/bin/python

import rospy
import json
from failure_detection.task_failure import TaskFailureDetector
from dynamic_skill_injection_msgs.srv import WorldState, TaskNetwork, TaskFailureNotification

def detect_task_failure():

    rospy.init_node('failure_detection_publisher', anonymous=False)

    detector = TaskFailureDetector()

    # We pass the bound detector methods world_state_callback and task_network_callback as the 
    # callback methods for each of these subscrubers, which update the detector instance's
    # member variables.
    rospy.Subscriber('dsi/current_world_state', WorldState,  detector.world_state_callback)
    rospy.Subscriber('dsi/current_task_network', TaskNetwork, detector.task_network_callback)

    pub = rospy.Publisher('dsi/task_failure_notification', TaskFailureNotification, queue_size=10)

    rate = rospy.Rate(10)  # 10hz 
    while not rospy.is_shutdown():
        tfn_msg = detector.check_failure()
        pub.publish(tfn_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        detect_task_failure()
    except rospy.ROSInterruptException:
        pass