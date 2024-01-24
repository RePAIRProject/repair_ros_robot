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
from xbot_msgs.msg import JointCommand, JointState
from repair_interface.srv import *
from sensor_msgs.msg import JointState as JointStateMsg


# Class definition
class MoveitClient:
    def __init__(self, node_handle, use_xbot, use_gazebo):
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

        self.xbot_state_sub = rospy.Subscriber(
            "/xbotcore/joint_states", JointState, self.xbot_state_callback
        )

        self.state_pub = rospy.Publisher("/joint_states", JointStateMsg, queue_size=10)

        self.nh.loginfo("Moveit client initialized")

    def init_moveit_client(self):
        self.robot = RobotCommander()
        # Initialize MoveGroupCommander for each arm
        self.move_group_arm_1 = MoveGroupCommander("arm_1", wait_for_servers=10)
        self.move_group_arm_2 = MoveGroupCommander("arm_2", wait_for_servers=10)

        # create service server
        self.service_server = self.nh.Service(
            "/move_arm_to_pose_py", MoveArmToPose, self.handle_move_arm_to_pose
        )

    def xbot_state_callback(self, msg: JointState):
        joint_state_msg = JointStateMsg()

        joint_state_msg.header.stamp = rospy.Time.now()

        joint_state_msg.name = msg.name
        joint_state_msg.position = msg.motor_position
        joint_state_msg.velocity = msg.motor_velocity

        self.state_pub.publish(joint_state_msg)

    def handle_move_arm_to_pose(self, req: MoveArmToPoseRequest):
        # get arm name
        arm = req.arm

        arm_name = "arm_1" if arm == 0 else "arm_2"

        # get pose
        pose = req.target_pose

        # send pose to arm
        res = self.send_pose_to_single_arm(pose, arm_name)

        resp = MoveArmToPoseResponse()
        resp.success = res
        resp.current_pose = self.get_current_pose(arm_name)

        # return response
        return resp

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
        _, plan, _, _ = move_group.plan()

        len_points = len(plan.joint_trajectory.points)

        # print points
        self.nh.loginfo(f'points in plan: {len(plan.joint_trajectory.points)}')

        if len_points == 0:
            self.nh.logwarn("No plan found")
            return False

        qs_sample = self.traj_utils.interpolate_joint_trajectory(plan, self.num_samples)

        # publish each point in the trajectory
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


if __name__ == "__main__":
    # Initialize the node
    roscpp_initialize(sys.argv)
    rospy.init_node("moveit_client_node")

    # get parameters
    use_xbot = rospy.get_param("~use_xbot", False)
    use_gazebo = rospy.get_param("~use_gazebo", False)

    # Create a MoveitClient object
    moveit_client = MoveitClient(rospy, use_xbot, use_gazebo)

    # Spin
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()