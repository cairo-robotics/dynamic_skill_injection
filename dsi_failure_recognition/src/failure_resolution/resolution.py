import json
import rospy
import copy
import pdb
import pprint

class FailureMapper(object):

    """
    Class that maps failed operators to their appropriate resolution action
    through handling requests for thefailure resolution service. It also
    manages state of the available resolution actions by offering a callback
    for a subscription to the /dsi/resolution_actions topic.
    """

    def __init__(self, resolution_actions=None):
        if(resolution_actions == None):
            self.resolution_actions = {}
        else:
            self.resolution_actions = resolution_actions

    def update_action_resolution_callback(self, action_resolution_msg):
        """
        Callback method to pass to a 'dsi/resolution_actions' subscriber.

        Parameters
        ----------
        action_resolution_msg : ActionResolution
            ActionResolution messsage from dynamic_skill_injection_msgs
            package.

        """
        new_resolution = copy.deepcopy(json.loads(action_resolution_msg.data))
        failed_action_key = new_resolution.keys()[0]
        resolution_action_key = new_resolution[failed_action_key].keys()[0]
        if(failed_action_key in self.resolution_actions.keys()):
            self.resolution_actions[failed_action_key][resolution_action_key] = new_resolution[failed_action_key][resolution_action_key]
        else:
            self.resolution_actions[failed_action_key] = new_resolution[failed_action_key]


    def action_resolution_callback(self, action_resolution_msg):
        """
        DEPRECATED - leaving this for now in case we do want to implement
            a full-scale dictionary replaced subscriber
        Callback method to pass to a 'dsi/resolution_actions' subscriber.

        Parameters
        ----------
        action_resolution_msg : ActionResolution
            ActionResolution messsage from dynamic_skill_injection_msgs
            package.

        """
        self.resolution_actions = json.loads(action_resolution_msg.data)

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
            Returns a formatted dictionary for the FailureResolution response
            with the corresponding failure resolution action. If no action is
            found, the resolution_action field will be None.

        """
        res = self._map_failure_to_resolution(json.loads(request.world_state), request.operator)
        return json.dumps(res)

    def _map_failure_to_resolution(self, world_state, operator):
        """
        Maps the current task failure to an appropriate resolution action.

        Parameters
        ----------
        world_state : dict
            Incoming FailureResolution.srv request JSON decoded world_state
            field representing current world state at time of request.

        operator : str
            Incoming operator that failed turning task plan execution.

        Returns
        ----------
        response : dict
            Returns a formatted dictionary for the FailureResolution response
            with the corresponding failure resolution action. If no action is
            found, the resolution_action field will be None.
        """
        if type(world_state) != dict:
            raise Exception("_map_failure_to_resolution expects world_state argument to be a dict.")
        if self.resolution_actions is not None and operator in self.resolution_actions.keys():
            resolutions = self.resolution_actions[operator]
            for resolution in resolutions.keys():
                failure_conditions = resolutions[resolution]["failure_conditions"]
                if self._check_world_state_for_conditions(world_state, failure_conditions) is True:
                    return {resolution: resolutions[resolution]}
        return {}

    def _check_world_state_for_conditions(self, world_state, conditions):
        """
        Compares the world_state with the required conditions of a failure
        resolution action.

        Parameters
        ----------
        world_state : dict
            State dictionary of an object of the WorldState message JSON
            encoded world state.
        conditions : dict
           Failure conditions required of the resolution action.

        Returns
        ----------
        status : bool
            Returns a boolean indicating if the current world state contains
            the failure conditions of the
            current resolution being tested.
        """
        # pdb.set_trace()
        status = False
        # Each key represents an object of the world state.
        for key in conditions.keys():
            # If the world state does not have the key, failure conditions are not met.
            if key not in world_state.keys():
                status = False
                break
            # get condition values and ws_value for object
            condition_value = conditions[key]
            ws_value = world_state[key]
            for key, value in condition_value.iteritems():
                # if type is list, compare sorted list of value in condition with value in ws
                if type(value) == list:
                    if key in ws_value and type(ws_value[key]) == list:
                        if sorted(value) == sorted(ws_value[key]):
                            status = True
                            continue
                        else:
                            status = False
                            break
                # otherwise just compare the values
                else:
                    if value == ws_value[key]:
                        status = True
                        continue
                    else:
                        status = False
                        break
        return status

class ResolutionLibrary():

    def __init__(self, resolution_actions=None):
        self.resolution_actions = resolution_actions

    def human_command_callback(self, human_command_msg):
        """
        Callback method to pass to a 'dsi/resolution_actions' subscriber.

        Parameters
        ----------
        human_command_msg : String
            String messsage from /human_command topic.

        """
        self.resolution_actions = json.loads(human_command_msg.data)
