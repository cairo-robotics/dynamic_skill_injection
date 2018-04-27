import json
from dynamic_skill_injection_msgs.msg import TaskFailureNotification

class FailureMapper(object):

    """
    Class that maps failed operators to their appropriate 
    """

    def __init__(self, resolution_actions):
        self.task_failure_notification_msg = None
        self.resolution_actions = resolution_actions

    def task_failure_notification_callback(self, task_failure_notification_msg):
        """
        Callback method to pass to TaskFailureNotification subscriber.

        Parameters
        ----------
        task_failure_notification : TaskFailureNotification
            TaskFailureNotification messsage from dynamic_skill_injection_msgs package.

        """
        self.task_failure_notification_msg = task_failure_notification_msg

    def map_failure_to_resolution(self):
        """
        Maps the current task failure to an appropriate resolution action.
        """
        failed_operator = self.task_failure_notification_msg.operator

        if failed_operator in self.resolution_actions.keys():
            resolutions = self.resolution_actions[failed_operator]
            


        return ()