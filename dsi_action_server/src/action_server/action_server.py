#! /usr/bin/env python2

import rospy
import actionlib
import pdb
import rosparam
import json
import rospkg

import tf

from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String
from geometry_msgs.msg import Pose


from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
import moveit_commander
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from movo_action_clients.gripper_action_client import GripperActionClient
from dsi_world_state.srv import world_state
from dsi_action_server.srv import action_service

from geometry_msgs.msg import PoseStamped, Pose

class actionServer(object):
    def __init__(self, configs="configs.json", methods="methodlib.json", open_g=.01, closed=.55):
        #TODO make config location as input / parameter server
        rospack = rospkg.RosPack()
        base_path = rospack.get_path("dsi_action_server")
        self.configs = json.load(open(base_path + "/config/configs.json"))
        self.method_lib = json.load(open(base_path + "/config/methodlib.json"))

        ''' initialize robot moveit '''
        self.scene = PlanningSceneInterface("base_link")
        self.robot = moveit_commander.RobotCommander()

        ''' set left arm setup '''
        self.group_left = moveit_commander.MoveGroupCommander("left_arm")
        self.group_right = moveit_commander.MoveGroupCommander("right_arm")

        ''' gripper setup'''
        self.lgripper = GripperActionClient('left')
        self.rgripper = GripperActionClient('right')
        self.gripper_open = open_g
        self.gripper_closed = closed

        ''' wait for gazebo services for teleport '''
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

        ''' world state service '''
        rospy.wait_for_service('/dsi/world_state_server')
        self.world_state_service = rospy.ServiceProxy('/dsi/world_state_server', world_state)

        ''' initialize action server service '''
        s = rospy.Service('dsi/action_server', action_service, self.action_service)

        rospy.loginfo("action server intitialized")



    def action_service(self, req):
        """
        the action service to be called by

        Parameters
        ----------
        """
        print req.action_request
        request = json.loads(req.action_request)
        print request

        if "method" in request:
            self.method_picker(request["method"])
            return "ran method"

        return "error"

    def method_picker(self, method):
        """
        grabs method from the loaded methodlib file

        Parameters
        ----------
        method: the method from the action config file to be used
        """
        #TODO wrap for errors and issues allow inbound services
        self.dict_method_parser(self.method_lib[method])


    def dict_method_parser(self, dict_method):
        #TODO
        """
        takes a method dictionary and sends them to the action parsers
        to be executed by the bot

        Parameters
        ----------
        dict_method: A dictionary of actions to be executed
        """
        for action in dict_method:
            if action["action"] == "joint_state":
                rospy.loginfo("joint state parser started")
                self.joint_state_parser(action)

            elif action["action"] == "pose_arm":
                rospy.loginfo("joint state parser started")
                self.pose_arm_parser(action)

            elif action["action"] == "set_gripper":
                rospy.loginfo("set gripper parser started")
                self.set_gripper_parser(action)

            elif action["action"] == "teleport":
                rospy.logwarn("teleport action not made yet")

            else:
                rospy.logwarn("method parser invalid action")

    def teleport_parser(self, teleport_dict):
        """
        takes in teleport parameters into dictionary
        Parameters
        ----------
        teleport_dict: dictionary of teleport parameters
        """
        tele_pose = Pose()


        self.teleport(tele_pose)

    def joint_state_parser(self, joint_state_dict):
        """
        pareses the joint state dictionary into a useable form for the joint
        state action

        Parameters
        ----------
        joint_state_dict: dictionary of joint state action data
        """
        try:
            joint_state_call = joint_state_dict["joint_state"]
        except Exception as e:
            rospy.logerr("joint state parser error: {}".format(e))
            return

        if joint_state_call not in self.configs:
            rospy.logerr("joint state doesn't exist")
            return
        joint_state = self.configs[joint_state_call]

        arm = "left"
        if arm in joint_state_dict:
            arm = joint_state_dict["arm"]

        self.set_joint(joint_state, arm)
        rospy.loginfo("joint state executed")




    def set_gripper_parser(self, set_gripper_dict):
        """
        parses the gripper action based on the gripper dictionary item


        Parameters
        ----------
        set_gripper_dict: parses the gripper action dictionary
        """
        arm = "left"
        if "arm" not in set_gripper_dict:
            rospy.logwarn("no arm set in gripper dictionary")
        else:
            arm = set_gripper_dict["arm"]
        if "position" not in set_gripper_dict:
            rospy.logerr("no postion in gripper action")
            return

        position = set_gripper_dict["position"]
        self.gripper_control(position, arm)
        rospy.loginfo("gripper {} action executed set to {}".format(arm, position))


    def pose_arm_parser(self, pose_arm_dict):
        """
        Takes in pose based dictionary action items and executes them

        Parameters
        ----------
        action_dict:
        """
        try:
            object = pose_arm_dict["object"]
            offset = pose_arm_dict["linear_offset"]
        except Exception as e:
            rospy.logerr("arm parser error: {}".format(e))
            return

        arm = "left"
        if "arm" in pose_arm_dict:
            arm = pose_arm_dict["arm"]

        self.pose_arm_wrapper(object, offset, arm)
        rospy.loginfo("pose arm executed")


    def pose_arm_wrapper(self, object, offset, arm="left"):
        """
        wraps the poase arm function to use world state objects

        Parameters()
        ----------
        objects: the object to set pose in relation too
        offset: linear translation of offset from object list[x,y,z]
        """
        obj_pose = self.read_object(object)

        obj_pose.position.x += offset[0]
        obj_pose.position.y += offset[1]
        obj_pose.position.z += offset[2]

        self.set_pose(obj_pose, arm)


    def read_object(self, object):
        """
        reads object pose info from the world state server

        Parameters
        ----------
        object: the object for who's desired information you would like

        Returns
        -------
        pose: returns pose object
        """
        world_state = json.loads(self.world_state_service('blank').world_state)
        #TODO check for objects existence
        position = world_state[object]["location_xyz"]
        orientation = world_state[object]["orientation_rpq"]
        quaternion = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2])

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        return pose

    def set_pose(self, pose, arm):
        """
        Choose an arm and a Pose to place it into

        Parameters
        ----------
        pose: ros Pose message showing the desired location
        arm: choice of arm to execute action
        """
        #TODO don't like this method of arm picking
        if arm == "left":
            self.group_left.set_pose_target(pose)
            self.group_left.execute(self.group_left.plan())

        elif arm == "right":
            self.group_right.set_pose_target(pose)
            self.group_right.execute(self.group_right.plan())

        else:
            rospy.logwarn("didn't pick a proper arm")

    def set_joint(self, joint, arm):
        """
        Choose and arm and a joint state to place it into

        Parameters
        ----------
        joint: list containing joint indo
        arm: choice of arm to execute action
        """
        if arm == "left":
            self.group_left.set_joint_value_target(joint)
            self.group_left.execute(self.group_left.plan())

        elif arm == "right":
            self.group_right.set_joint_value_target(joint)
            self.group_right.execute(self.group_right.plan())

        else:
            rospy.logwarn("didn't pick a proper arm")



    def gripper_control(self, position, gripper):
        """
        tells movo to open or close the gripper

        Parameters
        ----------
        position: open or closed hand position
        gripper: left or right gripper
        """
        if position == "open":
            if gripper == "left":
                self.lgripper.command(self.gripper_open)
            elif gripper == "right":
                self.rgripper.command(self.gripper_open)
            else:
                rospy.logwarn("gripper issue")

        elif position == "closed":
            if gripper == "left":
                self.lgripper.command(self.gripper_closed)
            elif gripper == "right":
                self.rgripper.command(self.gripper_closed)
            else:
                rospy.logwarn("gripper issue")

        else:
            rospy.logwarn("gripper issue")


    def teleport(self, location_pose, agent='movo'):
        """
        places movo in the desired location

        Parameters:
        -----------
        location_pose: Pose data type for desired location of movo
        """
        teleport_loc = ModelState()
        teleport_loc.model_name = agent
        teleport_loc.pose = location_pose
        self.set_model_state(teleport_loc)

    def dict_to_pose_msg(self, dict):
        """
        takes a json dictionary item and place into a pose message

        Parameters:
        -----------
        dict: dictionary item to be placed into a pose message
        """
        pose = Pose()
        pose.position.x = dict["position"]["x"]
        pose.position.y = dict["position"]["y"]
        pose.position.z = dict["position"]["z"]

        pose.orientation.x = dict["orientation"]["x"]
        pose.orientation.y = dict["orientation"]["y"]
        pose.orientation.z = dict["orientation"]["z"]
        pose.orientation.w = dict["orientation"]["w"]

        return pose



if __name__ == '__main__':


    test = actionServer()
    test_pose = test.dict_to_pose_msg(test.configs["test_pose1"])
    print test_pose
    test.teleport(test_pose)
    #test.set_pose(test_pose, "right")
    test.set_joint(test.configs["left_joint_stow"], "left")
    test.set_joint(test.configs["right_joint_stow"], "right")
    """
    while not rospy.is_shutdown():
        break
        result = lmove_group.moveToJointPosition(left_arm_joints, larm_const_stow, 0.05)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            print "sucess"
            break
    """

    pass
