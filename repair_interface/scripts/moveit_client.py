#!/usr/bin/env python3

# Importing necessary libraries
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
from moveit_commander.conversions import pose_to_list
import rospy
import tf
import sys

# Class definition
class MoveitClient:
    def __init__(self, node_handle):
        self.nh = node_handle
        self.init_moveit_client()

    def init_moveit_client(self):
        self.robot = RobotCommander()
        # Initialize MoveGroupCommander for each arm
        self.move_group_arm_1 = MoveGroupCommander("arm_1")
        self.move_group_arm_2 = MoveGroupCommander("arm_2")

    def send_pose_to_single_arm(self, pose, arm):
        '''
        Send a pose to a single arm
        :param pose: PoseStamped object
        :param arm: "ARM_1" or "ARM_2"
        '''
        # Choose the appropriate arm
        move_group = self.move_group_arm_1 if arm == "ARM_1" else self.move_group_arm_2
        move_group.set_pose_target(pose)
        # get plan
        # plan = move_group.plan()
        return move_group.go(wait=True)
    
    def compute_plan_to_single_arm(self, pose, arm):
        '''
        Compute a plan to a single arm
        :param pose: PoseStamped object
        :param arm: "ARM_1" or "ARM_2"
        '''
        # Choose the appropriate arm
        move_group = self.move_group_arm_1 if arm == "ARM_1" else self.move_group_arm_2
        move_group.set_pose_target(pose)
        # get plan
        plan = move_group.plan()
        return plan

    def send_pose_to_both_arms(self, arm_1_pose, arm_2_pose):
        # Set pose for each arm and execute
        self.move_group_arm_1.set_pose_target(arm_1_pose)
        success_arm_1 = self.move_group_arm_1.go(wait=True)
        self.move_group_arm_2.set_pose_target(arm_2_pose)
        success_arm_2 = self.move_group_arm_2.go(wait=True)
        return success_arm_1 and success_arm_2

    def move_to_home(self, arm):
        # Move the specified arm to its 'home' position
        move_group = self.move_group_arm_1 if arm == "ARM_1" else self.move_group_arm_2
        move_group.set_named_target("home")
        return move_group.go(wait=True)

    def get_arm_move_group_and_plan(self, arm):
        # Get move group and plan for the specified arm
        move_group = self.move_group_arm_1 if arm == "ARM_1" else self.move_group_arm_2
        plan = move_group.plan()
        return move_group, plan

    def convert_quat_to_rpy(self, pose):
        # Convert quaternion to roll, pitch, yaw
        quaternion = (pose.pose.orientation.x,
                      pose.pose.orientation.y,
                      pose.pose.orientation.z,
                      pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler  # Returns a tuple (roll, pitch, yaw)

if __name__ == "__main__":
    # Initialize the node
    roscpp_initialize(sys.argv)
    rospy.init_node("moveit_client_node")

    # Create a MoveitClient object
    moveit_client = MoveitClient(rospy)



