import rospy
import json
from dynamic_skill_injection_msgs.msg import TaskFailureNotification


class TaskFailureDetector(object):

    def __init__(self):
        self.world_state_msg = None
        self.task_network_msg = None

    def world_state_callback(self, world_state_msg):
        self.world_state_msg = world_state_msg

    def task_network_callback(self, task_network_msg):
        self.task_network_msg = task_network_msg

    def compare_entries(self, world_state, effect):
        pass

    def check_failure(self):
        status = False
        if self.world_state_msg is not None and self.task_network_msg is not None:
            world_state = json.loads(self.world_state_msg.world_state)
            effects_dict = json.loads(self.task_network_msg.effect)
            comparisons = []

            for key in effects_dict.keys():
                comparison = []
                comparison.append(effects_dict[key])

                if key in world_state.keys():
                    comparison.append(world_state[key])

                comparisons.append(comparison)

            rospy.loginfo(comparisons)

        tfn_msg = TaskFailureNotification()
        tfn_msg.task_name = "test_name"
        tfn_msg.operator = "operator_name"
        tfn_msg.status = True

        return tfn_msg

