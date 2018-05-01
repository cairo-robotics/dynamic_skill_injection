import json
from dynamic_skill_injection_msgs.msg import TaskFailureNotification

class FailureMapper(object):

    """
    Class that maps failed operators to their appropriate resolution action and handles requests for the
    failure resolution service. It also manages state of the available resolution actions by offering 
    a callback for a subscription to the /dsi/resolution_actions topic.
    """

    def __init__(self, resolution_actions=None):
        self.resolution_actions = resolution_actions

    def action_resolution_callback(self, action_resolution_msg):
        """
        Callback method to pass to TaskFailureNotification subscriber.

        Parameters
        ----------
        action_resolution_msg : ActionResolution
            ActionResolution messsage from dynamic_skill_injection_msgs package.

        """
        self.resolution_actions = action_resolution_msg

    def request_handler(self, request):
        """
        Handles the income request of the failure_resolution_server
        Parameters
        ----------
        request : FailureResolution
            Incoming FailureResolution.srv request 

        Returns
        ----------
        response : dict
            Returns a formatted dictionary for the FailureResolution response.

        """
        failed_operator = 
               resolution_action = self.map_failure_to_resolution(world_state, effects_dict)
        response = {
            "resolution_action": success
        }
        return response

    def map_failure_to_resolution(self, world_state):
        """
        Maps the current task failure to an appropriate resolution action.
        """
        failed_operator = self.task_failure_notification_msg.operator
        if failed_operator in self.resolution_actions.keys():
            resolutions = self.resolution_actions[failed_operator]
            for resolution in resolutions:
                failure_conditions = resolution["conditions"]
                if self.check_world_state_for_conditions(world_state, failure_conditionsl) is True:
                    return resolution
        return None

    def check_world_state_for_conditions(self, world_state, conditions) 
        """
        Compares the world_state with the required conditions of a failure resolution action.

        Parameters
        ----------
        world_state : dict
            State dictionary of an object of the WorldState message JSON encoded world state.
        conditions : dict
           Failure conditions required of the resolution action. 

        Returns
        ----------
        world_state : tuple (dict, bool)
            Returns a 2-tuple. The first element is a dictionary of the expected effects not present in world state.
            The second element is a boolean representing success or failure.

        """
        # TODO REFACTOR THIS IMPLEMENTATION TO CHECK FOR FAILURE CONDITIONS
        expected_effect = {}
        status = True
        for key in effect.keys():
            # If the key does not exist in the world state, there is an expected effect that introduces
            # a new key value pair to the world state for the given object. If this is not in the world state
            # this constitutes a task network operator failure.
            if key not in world_state.keys():
                expected_effect[key] = effect[key]
                status = False
                break
            effect_value = effect[key]
            ws_value = world_state[key]

            if type(effect_value) is list:
                # If the value is a list of list, we compare the sorted list so that the ordering does not matter.
                if len(effect_value) != 0 and type(effect_value[0]) is list:
                    if sorted(effect_value) != sorted(ws_value):
                        expected_effect[key] = effect_value
                        status = False
                else:
                    if effect_value != ws_value:
                        expected_effect[key] = effect_value
                        status = False
            else:
                if effect_value != ws_value:
                    expected_effect[key] = effect_value
                    status = False

        return (expected_effect, status)