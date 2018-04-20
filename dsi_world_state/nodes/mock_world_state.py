#!/usr/bin/python

import rospy
import json
from dynamic_skill_injection_msgs.msg import WorldState

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
    }
}


def current_world_state(world_state):
    pub = rospy.Publisher('dsi/current_world_state', WorldState, queue_size=10)
    rospy.init_node('world_state_publisher', anonymous=False)
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
