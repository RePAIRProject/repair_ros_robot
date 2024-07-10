#!/usr/bin/env python3

import os
from typing import Union
from enum import Enum

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

from transform_utils import TransformUtils
import tf_conversions

from traj_utils import TrajectoryUtils

from dawn_ik.msg import IKGoal
from repair_interface.srv import *

from xbot_msgs.msg import JointCommand


class ARM_ENUM(Enum):
    ARM_1 = 0
    ARM_2 = 1


class MotionPlanner:
    def __init__(self):

        # transform utils
        self.tu = TransformUtils()
        self.traj_utils = TrajectoryUtils()

        # publish joint state
        self.joint_state_pub = rospy.Publisher(
            "joint_states",
            JointState,
            queue_size=10,
        )

        succ, self.kdl_tree = treeFromParam("robot_description")
        if not succ:
            raise RuntimeError("Failed to parse URDF")
        
        self.sliding_guide_chain = self.kdl_tree.getChain(
            "world",
            "dummy_prismatic_link",
        )

        self.left_arm_chain = self.kdl_tree.getChain(
            "dummy_prismatic_link",
            "left_hand_v1_2_research_grasp_link",
        )
        self.right_arm_chain = self.kdl_tree.getChain(
            "dummy_prismatic_link",
            "right_hand_v1_2_research_grasp_link",
        )

        self.left_arm_grasp_link = "left_hand_v1_2_research_grasp_link"
        self.right_arm_grasp_link = "right_hand_v1_2_research_grasp_link"

        # ik solver
        self.sliding_guide_ik_solver = kdl.ChainIkSolverPos_LMA(self.sliding_guide_chain)
        self.left_arm_ik_solver = kdl.ChainIkSolverPos_LMA(self.left_arm_chain)
        self.right_arm_ik_solver = kdl.ChainIkSolverPos_LMA(self.right_arm_chain)

        self.dawn_ik_arm_1_goal_pub = rospy.Publisher(
            "/arm_1/dawn_ik_solver/ik_goal", IKGoal, queue_size=5
        )

        self.dawn_ik_arm_2_goal_pub = rospy.Publisher(
            "/arm_2/dawn_ik_solver/ik_goal", IKGoal, queue_size=5
        )

        # service
        self.move_arm_to_pose_service = rospy.Service(
            "/motion_planner/dawnik",
            MoveArmToPose,
            self.handle_move_arm_to_pose,
        )

        self.sliding_guide_joint = "j_sliding_guide"

        self.right_arm_joints = [
            "j_torso_1",
            "j_arm_2_1",
            "j_arm_2_2",
            "j_arm_2_3",
            "j_arm_2_4",
            "j_arm_2_5",
            "j_arm_2_6",
            "j_arm_2_7",
        ]

        self.left_arm_joints = [
            "j_torso_1",
            "j_arm_1_1",
            "j_arm_1_2",
            "j_arm_1_3",
            "j_arm_1_4",
            "j_arm_1_5",
            "j_arm_1_6",
            "j_arm_1_7",
        ]

        self.xbot_command_pub = rospy.Publisher(
            "/xbotcore/command",
            JointCommand,
            queue_size=10,
        )

    def handle_move_arm_to_pose(self, req: MoveArmToPoseRequest):
        arm = req.arm
        rospy.loginfo(f"Received move arm {arm+1} to pose request")
        pose = req.target_pose

        response = MoveArmToPoseResponse()

        # plan
        succ, plan = self.plan(pose, arm)

        if not succ:
            response.success = False
            return response

        # execute
        res = self.execute(plan, arm)

        if not res:
            response.success = False
            return response
        
        response.success = True

        return response

    def get_current_pose(self, arm: ARM_ENUM, target_frame:str = "world") -> PoseStamped:

        pose = PoseStamped()

        if arm == ARM_ENUM.ARM_1.value:
            link = self.left_arm_grasp_link
        elif arm == ARM_ENUM.ARM_2.value:
            link = self.right_arm_grasp_link
        else:
            raise ValueError("Invalid arm")

        pose.header.frame_id = link
        pose.header.stamp = rospy.Time(0)

        pose_in_world = self.tu.transform_pose_with_retries(pose, target_frame)

        return pose_in_world

    def check_reachability(self, pose: PoseStamped, arm: ARM_ENUM):
        
        # get joint state
        joint_state = rospy.wait_for_message(
            "/joint_states",
            JointState,
            timeout=1.0,
        )

        if joint_state is None:
            raise RuntimeError("Failed to get joint state")

        if arm == ARM_ENUM.ARM_1.value:
            joints = self.left_arm_joints
            n_joints = len(self.left_arm_joints)
        elif arm == ARM_ENUM.ARM_2.value:
            joints = self.right_arm_joints
            n_joints = len(self.right_arm_joints)
        else:
            raise ValueError("Invalid arm")

        q_init = kdl.JntArray(n_joints)

        for i, joint in enumerate(joints):
            q_init[i] = joint_state.position[joint_state.name.index(joint)]

        # transform pose to "dummy_prismatic_link"
        pose = self.tu.transform_pose_with_retries(pose, "dummy_prismatic_link")

        pose = tf_conversions.fromMsg(pose.pose)

        return self.compute_ik(pose, arm, q_init)[0]

    def compute_ik(self, pose, arm, q_init=None):
        if arm == ARM_ENUM.ARM_1.value:
            ik_solver = self.left_arm_ik_solver
            n_joints = len(self.left_arm_joints)
        elif arm == ARM_ENUM.ARM_2.value:
            ik_solver = self.right_arm_ik_solver
            n_joints = len(self.right_arm_joints)
        else:
            raise ValueError("Invalid arm")

        if q_init is None:
            q_init = kdl.JntArray(n_joints)

        q_result = kdl.JntArray(n_joints)

        # compute ik
        if ik_solver.CartToJnt(q_init, pose, q_result) >= 0:
            rospy.loginfo("IK solution found")
            return True, q_result
        else:
            rospy.logwarn("IK solution not found")
            return False, None

    def get_sliding_guide_ik(self, pose, q_init=None):
        # get joint state
        joint_state = rospy.wait_for_message(
            "/joint_states",
            JointState,
            timeout=1.0,
        )

        if joint_state is None:
            raise RuntimeError("Failed to get joint state")
        
        n_joints = 1
        q_init = kdl.JntArray(n_joints)

        q_init[0] = joint_state.position[joint_state.name.index(self.sliding_guide_joint)]

        q_result = kdl.JntArray(n_joints)

        pose = tf_conversions.fromMsg(pose)

        # compute ik
        if self.sliding_guide_ik_solver.CartToJnt(q_init, pose, q_result) >= 0:
            rospy.loginfo("IK solution found")
            return True, q_result
        else:
            rospy.logwarn("IK solution not found")
            return False, None

    def plan(self, pose, arm: ARM_ENUM) -> Union[bool, list]:

        # check reachability
        reachable = self.check_reachability(pose, arm)

        if not reachable:
            rospy.logerr("Pose not reachable")
            return False, None

            # TODO: move the sliding guide
            # sliding_guide_pose = self.tu.get_link_pose("dummy_prismatic_link")
            # # assumption - the sliding guide axis is Y in world frame
            # sliding_guide_pose.pose.position.y += (pose.pose.position.y - sliding_guide_pose.pose.position.y) / 2

            # # compute ik
            # succ, q_result = self.get_sliding_guide_ik(sliding_guide_pose.pose)

            # if succ:
            #     jc = JointCommand()
            #     jc.header.stamp = rospy.Time.now()
            #     jc.name = [self.sliding_guide_joint]
            #     jc.position = [q_result[0]]
            #     jc.ctrl_mode = [1]

            #     self.xbot_command_pub.publish(jc)
            #     rospy.sleep(1)

            # else:
            #     rospy.logerr("Failed to move sliding guide")
            #     return False, None



        pose_in_world = self.get_current_pose(arm, "world")

        # compute trajectory
        lin_points, rot_quats = self.traj_utils.compute_trajectory(pose_in_world, pose)

        # self.traj_utils.publish_pose_array(lin_points, rot_quats)

        return True, [lin_points, rot_quats]

    def execute(self, plan, arm: ARM_ENUM):
        pub_rate = rospy.Rate(100)

        if arm == ARM_ENUM.ARM_1.value:
            dawn_ik_goal_pub = self.dawn_ik_arm_1_goal_pub
        elif arm == ARM_ENUM.ARM_2.value:
            dawn_ik_goal_pub = self.dawn_ik_arm_2_goal_pub

        for position, quat in zip(plan[0], plan[1]):
            dawn_ik_goal = IKGoal()
            dawn_ik_goal.header.stamp = rospy.Time.now()
            dawn_ik_goal.header.frame_id = "world"
            dawn_ik_goal.mode = IKGoal.MODE_1 + IKGoal.MODE_2
            dawn_ik_goal.m1_x = position[0]
            dawn_ik_goal.m1_y = position[1]
            dawn_ik_goal.m1_z = position[2]
            dawn_ik_goal.m1_limit_dist = 0.1
            dawn_ik_goal.m1_weight = 4
            dawn_ik_goal.m2_x = quat[0]
            dawn_ik_goal.m2_y = quat[1]
            dawn_ik_goal.m2_z = quat[2]
            dawn_ik_goal.m2_w = quat[3]
            dawn_ik_goal.m2_weight = 1

            dawn_ik_goal_pub.publish(dawn_ik_goal)

            pub_rate.sleep()

        rospy.sleep(2)
        # send idle goal
        dawn_ik_goal = IKGoal()
        dawn_ik_goal.mode = IKGoal.MODE_0
        dawn_ik_goal_pub.publish(dawn_ik_goal)
        rospy.loginfo("Finished executing plan")

        return True


if __name__ == "__main__":
    rospy.init_node("motion_planner")
    mp = MotionPlanner()
    rospy.spin()
