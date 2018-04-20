#!usr/bin/python

#!/usr/bin/python

import rospy
import json
from failure_detection.task_failure import TaskFailureRecognizer



def current_world_state(world_state):
    pub = rospy.Publisher('dsi/current_world_state', WorldState, queue_size=10)
    rospy.init_node('world_state_pub', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        ws_msg = WorldState()
        ws_msg.world_state = json.dumps(world_state)
        pub.publish(ws_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        current_world_state(world_state)
    except rospy.ROSInterruptException:
        pass