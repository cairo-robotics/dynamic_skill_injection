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


from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
import moveit_commander
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from movo_action_clients.gripper_action_client import GripperActionClient

from geometry_msgs.msg import PoseStamped, Pose

class actionLibrary(object):
    def __init__(self):
        #TODO make config location as input / parameter server
        rospack = rospkg.RosPack()
        base_path = rospack.get_path("dsi_action_server")
        self.configs = json.load(open(base_path + "/config/prelim.json"))

        rospy.init_node("beta_action_server")

        self.scene = PlanningSceneInterface("base_link")
        self.robot = moveit_commander.RobotCommander()

        self.group_left = moveit_commander.MoveGroupCommander("left_arm")
        self.group_right = moveit_commander.MoveGroupCommander("right_arm")

        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)


    def set_pose(self, pose, arm):
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
        if arm == "left":
            self.group_left.set_joint_value_target(joint)
            self.group_left.execute(self.group_left.plan())

        elif arm == "right":
            self.group_right.set_joint_value_target(joint)
            self.group_right.execute(self.group_right.plan())

        else:
            rospy.logwarn("didn't pick a proper arm")

    def teleport(self, location_pose):
        teleport_loc = ModelState()
        teleport_loc.model_name = 'movo'
        teleport_loc.pose = location_pose
        self.set_model_state(teleport_loc)

    def dict_to_pose_msg(self, dict):
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


    test = actionLibrary()
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
