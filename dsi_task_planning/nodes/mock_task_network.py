#!/usr/bin/python

import rospy
import json
from dynamic_skill_injection_msgs.msg import TaskNetwork

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


def current_planning_step(task_name, operator, effect):
    pub = rospy.Publisher('dsi/current_task_network', TaskNetwork, queue_size=10)
    rospy.init_node('task_network_publisher', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        tn_msg = TaskNetwork()
        tn_msg.task_name = task_name
        tn_msg.operator = operator
        tn_msg.effect = json.dumps(effect)
        pub.publish(tn_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        current_planning_step("fetch_mug", "search", effect)
    except rospy.ROSInterruptException:
        pass