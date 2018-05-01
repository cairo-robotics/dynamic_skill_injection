import json
# from dynamic_skill_injection_msgs.msg import TaskFailureNotification


class TaskFailureDetector(object):

    """
    Class that handles the failure notification service. It detects whether a current task network operator 
    fails based on the difference between the expected effects of the operator and the expressed fields in the
    current world state.
    """

    def handle_request(self, request):
        """
        Handles the income request of the failure_notification_server
        Parameters
        ----------
        request : TaskFailure
            Incoming TaskFailure.srv request 

        Returns
        ----------
        response : dict
            Returns a formatted dictionary for the FailureNotification.srv response.

        """
        world_state = json.loads(request.world_state)
        effects_dict = json.loads(request.effect)
        success, expected_effect = self.check_failure(world_state, effects_dict)
        response = {
            "success": success,
            "expected_effect": json.dumps(expected_effect)
        }
        return response

    def compare_entries(self, world_state, effect):
        """
        Compares the world_state with the required effects of the current task operator.

        Parameters
        ----------
        world_state : dict
            World state dictionary of the WorldState message JSON string world state field.
        effect : dict
            Dictionary of expected effects of the TaskNetwork message JSON string effect field.

        Returns
        ----------
        world_state : tuple (dict, bool)
            Returns a 2-tuple. The first element is a dictionary of the expected effects not present in world state.
            The second element is a boolean representing success or failure.

        """
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

    def check_failure(self, world_state, effects):
        """
        Iterative compares expected effects designated in the request.effects field with the 
        requestworld_state field. These are decoded JSON dictionaries of dictionaries.

        Parameters
        ----------
        world_state : dict
            State dictionary of an object of the FailureNotification message JSON encoded world state.
        effects : dict
            State dictionary of an object of the FailureNotification message JSON encoded effects.
        Returns
        -------
        : tuple
            Tuple with failure_status as first element and the expected_effect as the second
        """
        failure_status = True
        expected_effect = {}

        for key in effects.keys():
            eff = effects[key]
            if key in world_state.keys():
                ws = world_state[key]
                expected, status = self.compare_entries(world_state[key], effects[key])
                expected_effect[key] = expected
                if failure_status != False:
                    failure_status = status
            # If the world state does not have the expected effect, we have a failure.
            else:
                expected_effect[key] = effects[key]
                failure_status = False

        return (failure_status, expected_effect)



# DEPRECATED AS FAILURE NOTIFCATION IS NOW A SERVICE
# class TaskFailureDetector(object):

#     """
#     Class that detects whether a current task network operator fails based on the difference between
#     the expected effects of the operator are expressed in the currently published world state.
#     """

#     def __init__(self):
#         self.world_state_msg = None
#         self.task_network_msg = None

#     def world_state_callback(self, world_state_msg):
#         """
#         Callback method to pass to WorldState subscriber.

#         Parameters
#         ----------
#         world_state_msg : WorldState
#             WorldState messsage from dynamic_skill_injection_msgs package.

#         """
#         self.world_state_msg = world_state_msg

#     def task_network_callback(self, task_network_msg):
#         """
#         Callback method to pass to TaskNetwork subscriber.

#         Parameters
#         ----------
#         task_network_msg : WorldState
#             TaskNetwork messsage from dynamic_skill_injection_msgs package.

#         """
#         self.task_network_msg = task_network_msg

#     def compare_entries(self, world_state, effect):
#         """
#         Compares the world_state with the required effects of the current task operator.

#         Parameters
#         ----------
#         world_state : dict
#             State dictionary of an object of the WorldState message JSON encoded world state.
#         effect : dict
#             State dictionary of an object of the TaskNetwork message JSON encoded effect.

#         Returns
#         ----------
#         world_state : tuple (dict, bool)
#             Returns a 2-tuple. The first element is a dictionary of the expected effects not present in world state.
#             The second element is a boolean representing success or failure.

#         """
#         expected_effect = {}
#         status = True
#         for key in effect.keys():
#             # If the key does not exist in the world state, there is an expected effect that introduces
#             # a new key value pair to the world state for the given object. If this is not in the world state
#             # this constitutes a task network operator failure.
#             if key not in world_state.keys():
#                 expected_effect[key] = effect[key]
#                 status = False
#                 break
#             effect_value = effect[key]
#             ws_value = world_state[key]

#             if type(effect_value) is list:
#                 # If the value is a list of list, we compare the sorted list so that the ordering does not matter.
#                 if len(effect_value) != 0 and type(effect_value[0]) is list:
#                     if sorted(effect_value) != sorted(ws_value):
#                         expected_effect[key] = effect_value
#                         status = False
#                 else:
#                     if effect_value != ws_value:
#                         expected_effect[key] = effect_value
#                         status = False
#             else:
#                 if effect_value != ws_value:
#                     expected_effect[key] = effect_value
#                     status = False

#         return (expected_effect, status)

#     def check_failure(self):
#         """
#         Iterative compares expected effects designated in the task_network_msg effect field with the 
#         world_state_msg world_state field. These are decoded JSON dictionaries of dictionaries.

#         Returns
#         -------
#         tfn_msg : TaskFailureNotification
#             Returns as TaskFailureNotification message.
#         """
#         failure_status = True
#         expected_effect = {}
#         tfn_msg = TaskFailureNotification()
#         if self.world_state_msg is not None and self.task_network_msg is not None:
#             world_state = json.loads(self.world_state_msg.world_state)
#             effects_dict = json.loads(self.task_network_msg.effect)

#             for key in effects_dict.keys():
#                 eff = effects_dict[key]
#                 if key in world_state.keys():
#                     ws = world_state[key]
#                     expected, status = self.compare_entries(world_state[key], effects_dict[key])
#                     expected_effect[key] = expected
#                     if failure_status != False:
#                         failure_status = status
#                 # If the world state does not have the expected effect, we have a failure.
#                 else:
#                     expected_effect[key] = effects_dict[key]
#                     failure_status = False

#             tfn_msg.task_name = self.task_network_msg.task_name
#             tfn_msg.operator = self.task_network_msg.operator
#             tfn_msg.success = failure_status
#             tfn_msg.expected_effect = json.dumps(expected_effect)
#             tfn_msg.world_state = self.world_state_msg.world_state

#         return tfn_msg
