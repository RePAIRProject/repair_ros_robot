#!/usr/bin/env python3

# Importing necessary libraries
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
from moveit_commander.conversions import pose_to_list
import rospy
import tf
import sys
from traj_utils import TrajectoryUtils
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from xbot_msgs.msg import JointCommand


# Class definition
class MoveitClient:
    def __init__(self, node_handle):
        self.nh = node_handle
        self.init_moveit_client()

    def init_moveit_client(self):
        self.robot = RobotCommander()
        # Initialize MoveGroupCommander for each arm
        self.move_group_arm_1 = MoveGroupCommander("arm_1", wait_for_servers=10)
        self.move_group_arm_2 = MoveGroupCommander("arm_2")

    def send_pose_to_single_arm(self, pose, arm):
        """
        Send a pose to a single arm
        :param pose: PoseStamped object
        :param arm: "arm_1" or "arm_2"
        """
        # Choose the appropriate arm
        move_group = self.move_group_arm_1 if arm == "arm_1" else self.move_group_arm_2
        move_group.set_pose_target(pose)
        # get plan
        # plan = move_group.plan()
        return move_group.go(wait=True)

    def compute_plan_to_single_arm(self, pose, arm):
        """
        Compute a plan to a single arm
        :param pose: PoseStamped object
        :param arm: "ARM_1" or "ARM_2"
        """
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
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler  # Returns a tuple (roll, pitch, yaw)

    def get_current_pose(self, arm):
        # Get the current pose of the specified arm
        move_group = self.move_group_arm_1 if arm == "arm_1" else self.move_group_arm_2
        return move_group.get_current_pose()


if __name__ == "__main__":
    # Initialize the node
    roscpp_initialize(sys.argv)
    rospy.init_node("moveit_client_node")

    # Create a MoveitClient object
    moveit_client = MoveitClient(rospy)

    # Create a TrajectoryUtils object
    traj_utils = TrajectoryUtils()

    arm_name = "arm_2"

    # publisher
    pub = rospy.Publisher(
        f"/{arm_name}_trajectory_controller/command", JointTrajectory, queue_size=10
    )

    xbot_pub = rospy.Publisher(
        "/xbotcore/command", JointCommand, queue_size=10
    )

    # get current pose
    current_pose = moveit_client.get_current_pose(arm_name).pose

    # create a target pose
    target_pose = current_pose
    target_pose.position.x += 0.0
    target_pose.position.y += 0.0
    target_pose.position.z += 0.2

    # get plan
    res, plan, time, ec = moveit_client.compute_plan_to_single_arm(
        target_pose, arm_name
    )

    # print number of points in plan
    print(
        f"Res: {res} Number of points in plan: {len(plan.joint_trajectory.points)} \
            Time: {time}, Error Code: {ec}"
    )

    if res:
        qs_sample = traj_utils.interpolate_joint_trajectory(plan, 100)

        # publish each point in the trajectory
        for q in qs_sample:
            # # create a joint trajectory point
            # point = JointTrajectoryPoint()
            # point.positions = q
            # point.time_from_start = rospy.Duration(0.1)

            # # create a joint trajectory
            # traj = JointTrajectory()
            # traj.joint_names = plan.joint_trajectory.joint_names
            # traj.points.append(point)

            # # publish the joint trajectory
            # pub.publish(traj)
            # rospy.sleep(0.1)

            # create a joint command
            joint_command = JointCommand()
            joint_command.ctrl_mode = [1, 1, 1, 1, 1, 1, 1]
            joint_command.name = plan.joint_trajectory.joint_names
            joint_command.position = q
            joint_command.header.stamp = rospy.Time.now()

            # publish the joint command
            xbot_pub.publish(joint_command)

        print("Done!")
