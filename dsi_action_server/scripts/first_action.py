#! /usr/bin/env python2

import rospy
import actionlib

from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
import moveit_commander
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from movo_action_clients.gripper_action_client import GripperActionClient



class actionClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.lmove_group = MoveGroupInterface("left_arm", "base_link")
        self.rmove_group = MoveGroupInterface("right_arm", "base_link")
        self.lmove_group.setPlannerId("RRTConnectkConfigDefault")
        self.rmove_group.setPlannerId("RRTConnectkConfigDefault")

        self._upper_body_joints = ["right_shoulder_pan_joint",
                                        "right_shoulder_lift_joint",
                                        "right_arm_half_joint",
                                        "right_elbow_joint",
                                        "right_wrist_spherical_1_joint",
                                        "right_wrist_spherical_2_joint",
                                        "right_wrist_3_joint",
                                        "left_shoulder_pan_joint",
                                        "left_shoulder_lift_joint",
                                        "left_arm_half_joint",
                                        "left_elbow_joint",
                                        "left_wrist_spherical_1_joint",
                                        "left_wrist_spherical_2_joint",
                                        "left_wrist_3_joint",
                                        "linear_joint",
                                        "pan_joint",
                                        "tilt_joint"]
        self._right_arm_joints = ["right_shoulder_pan_joint",
                                        "right_shoulder_lift_joint",
                                        "right_arm_half_joint",
                                        "right_elbow_joint",
                                        "right_wrist_spherical_1_joint",
                                        "right_wrist_spherical_2_joint",
                                        "right_wrist_3_joint"]
        self._left_arm_joints = ["left_shoulder_pan_joint",
                                        "left_shoulder_lift_joint",
                                        "left_arm_half_joint",
                                        "left_elbow_joint",
                                        "left_wrist_spherical_1_joint",
                                        "left_wrist_spherical_2_joint",
                                        "left_wrist_3_joint"]
        self.tucked = [-1.6,-1.5,0.4,-2.7,0.0,0.5, -1.7,1.6,1.5,-0.4,2.7,0.0,-0.5,1.7, 0.04, 0, 0]
        self.constrained_stow =[-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, 1.0, 2.6, -2.0, 0.0, -2.0, 0.0, 0.0, -1.0, 0.42, 0, 0]
        self.larm_const_stow = [2.6, -2.0, 0.0, -2.0, 0.0, 0.0, 1.0]
        self.rarm_const_stow = [-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, -1.0]

    def test_move(self):
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(self._upper_body_joints, self.constrained_stow, 0.05)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return


if __name__ == '__main__':
    rospy.init_node("test_action_sever")

    is_sim = rospy.get_param("~sim",False)
    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')

    if (is_sim):
        rospy.wait_for_message('/sim_initialized',Bool)


    move_base = MoveBaseActionClient(sim=is_sim,frame="odom")


    first_run = actionClient()
    first_run.test_move
