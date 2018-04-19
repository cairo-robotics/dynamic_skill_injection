import rospy
import json
# from dynamic_skill_injection_msgs.msg import WorldState, TaskPlanState


class Recognition(object):

    def check_failure(prev_state_msg, curr_state_msg, task_plan_state_msg):
        prev_state = json.loads(prev_state_msg.world_state_json)
        curr_state = json.loads(curr_state_msg.world_state_json)
        effects = task_plan_state_msg.effects

        for effect in effects:
            if prev_state[effect]

