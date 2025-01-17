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
from repair_interface.srv import *
from xbot_msgs.msg import JointCommand, JointState
from sensor_msgs.msg import JointState as JointStateMsg

# Class definition for MoveitClient
class MoveitClient:
    def __init__(self, node_handle,  use_xbot, use_gazebo):
        self.nh = node_handle
        self.init_moveit_client()
        self.traj_utils = TrajectoryUtils()

        self.num_samples = 500

        self.use_xbot = use_xbot
        self.use_gazebo = use_gazebo

        self.arm_1_pub = rospy.Publisher(
            "/arm_1_trajectory_controller/command", JointTrajectory, queue_size=10
        )

        self.arm_2_pub = rospy.Publisher(
            "/arm_2_trajectory_controller/command", JointTrajectory, queue_size=10
        )

        self.xbot_pub = rospy.Publisher(
            "/xbotcore/command", JointCommand, queue_size=10
        )

        self.nh.loginfo("Moveit client initialized")

    def init_moveit_client(self):
        self.robot = RobotCommander()
        self.move_group_arm_1 = MoveGroupCommander("arm_1", wait_for_servers=10)
        self.move_group_arm_2 = MoveGroupCommander("arm_2", wait_for_servers=10)

        self.service_server = self.nh.Service(
            "/move_arm_to_pose_py", MoveArmToPose, self.handle_move_arm_to_pose
        )

    def handle_move_arm_to_pose(self, req: MoveArmToPoseRequest):
        arm = req.arm
        arm_name = "arm_1" if arm == 0 else "arm_2"
        pose = req.target_pose
        res = self.send_pose_to_single_arm(pose, arm_name)

        resp = MoveArmToPoseResponse()
        resp.success = res
        resp.current_pose = self.get_current_pose(arm_name)
        return resp

    def send_pose_to_single_arm(self, pose, arm):
        move_group = self.move_group_arm_1 if arm == "arm_1" else self.move_group_arm_2
        move_group.set_pose_target(pose)
        _, plan, _, _ = move_group.plan()
        len_points = len(plan.joint_trajectory.points)

        self.nh.loginfo(f'points in plan: {len(plan.joint_trajectory.points)}')

        if len_points == 0:
            self.nh.logwarn("No plan found")
            return False

        qs_sample = self.traj_utils.interpolate_joint_trajectory(plan, self.num_samples)

        for i in range(len(qs_sample)):
            if self.use_xbot:
                self.publish_to_xbot(plan.joint_trajectory.joint_names, qs_sample[i])
            if self.use_gazebo:
                print(f"Publishing to gazebo: {arm}")
                self.pubslish_to_gazebo(
                    arm, plan.joint_trajectory.joint_names, qs_sample[i]
                )
            rospy.sleep(0.01)
        return True

    def publish_to_xbot(self, names, positions):
        # create a joint command
        joint_command = JointCommand()
        joint_command.ctrl_mode = [1, 1, 1, 1, 1, 1, 1]
        joint_command.name = names
        joint_command.position = positions
        joint_command.header.stamp = rospy.Time.now()

        # publish the joint command
        self.xbot_pub.publish(joint_command)

    def pubslish_to_gazebo(self, arm_name, names, positions):
        # create a joint trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(0.1)

        # create a joint trajectory
        traj = JointTrajectory()
        traj.joint_names = names
        traj.points.append(point)

        # publish the joint trajectory
        if arm_name == "arm_1":
            self.arm_1_pub.publish(traj)
        else:
            self.arm_2_pub.publish(traj)

    def get_current_pose(self, arm):
        # Get the current pose of the specified arm
        move_group = self.move_group_arm_1 if arm == "arm_1" else self.move_group_arm_2
        return move_group.get_current_pose()
    # Additional MoveIt related methods (e.g., move_to_home, get_current_pose, etc.)

if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node("moveit_client_node")

    # get parameters
    use_xbot = rospy.get_param("~use_xbot", False)
    use_gazebo = rospy.get_param("~use_gazebo", False)

    # Create a MoveitClient object
    moveit_client = MoveitClient(rospy, use_xbot, use_gazebo)

    rospy.spin()
