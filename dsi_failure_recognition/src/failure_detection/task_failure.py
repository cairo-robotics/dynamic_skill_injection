import rospy
import json
from dynamic_skill_injection_msgs.msg import TaskFailureNotification


class TaskFailure(object):

    def check_failure(world_state_msg, task_network_msg):
        world_state = json.loads(world_state_msg.world_state_json)
        effects_dict = json.loads(task_network_msg.effects)

        comparisons = []

        for key in effects_dict.keys():
            comparison = []
            comparison.append()

            if key in world_state.keys():
                comparison.append(world_state[key])


