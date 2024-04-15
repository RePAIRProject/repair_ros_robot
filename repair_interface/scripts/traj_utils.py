import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from scipy.spatial.transform import Rotation, Slerp
from moveit_msgs.msg import RobotTrajectory


class TrajectoryUtils:
    def __init__(self):
        self.pose_array_pub = rospy.Publisher(
            "/tu_pose_array", PoseArray, queue_size=1
        )
        rospy.sleep(1)

        # interpolation params
        self.num_points = 500

    def publish_pose_array(self, lin_points, rotations, frame_id="world"):
        # create a pose array
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = rospy.Time.now()

        # add the points to the pose array
        for point, quat in zip(lin_points, rotations):
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            pose_array.poses.append(pose)

        # publish the pose array
        self.pose_array_pub.publish(pose_array)

    def interpolate_joint_trajectory(self, moveit_plan: RobotTrajectory, num_points=100):
        """
        Interpolates a joint trajectory

        Inputs:
            moveit_plan: the moveit plan to interpolate

        Outputs:
            joint_trajectory: the interpolated joint trajectory
        """
        # get the joint names
        joint_names = moveit_plan.joint_trajectory.joint_names
        # get the joint positions
        moveit_points = moveit_plan.joint_trajectory.points
        joint_positions = np.array(
            [point.positions for point in moveit_points]
        )
        time_stamps = np.array(
            [point.time_from_start.to_sec() for point in moveit_points]
        )
        # Define your robot's dynamic parameters
        joint_velocities = np.array(
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Joint velocities (replace with actual values)
        joint_accelerations = np.array(
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Joint accelerations (replace with actual values)

        # path = ta.SplineInterpolator(time_stamps, joint_positions)
        # pc_vel = constraint.JointVelocityConstraint(joint_velocities)
        # pc_acc = constraint.JointAccelerationConstraint(joint_accelerations)
        # # Create a parametrization of the path
        # instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        # jnt_traj = instance.compute_trajectory()
        # # Sample the trajectory for joint positions
        # t_sample = np.linspace(0, jnt_traj.duration, num_points)
        # # Evaluate the trajectory
        # qs_sample = jnt_traj(t_sample)

        ss_waypoints = np.linspace(0, 1, len(joint_positions))
        path = ta.SplineInterpolator(ss_waypoints, joint_positions)

        t_sample = np.linspace(0, path.duration, num_points)
        qs_sample = path(t_sample)

        return qs_sample

    def compute_trajectory(
            self,
            current_pose: PoseStamped,
            target_pose: PoseStamped,
            lin_waypoints: np.ndarray = None,
            rot_waypoints: np.ndarray = None,
    ):
        """
        Computes a trajectory from the current pose to the given pose

        Inputs:
            current_pose: the current pose
            target_pose: the target pose
            lin_waypoints: linear waypoints as ndarray to add to the trajectory
            rot_waypoints: ndarray of rpy rotations to add to the trajectory

        Outputs:
            lin_points: the linear points of the trajectory
            rot_quats: the rotational quaternions of the trajectory
        """

        start_point = np.array(
            [
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
            ]
        )

        end_point = np.array(
            [
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
            ]
        )

        lin_x = np.array([start_point[0]])
        lin_y = np.array([start_point[1]])
        lin_z = np.array([start_point[2]])

        # add waypoints if given in between start and end point
        if lin_waypoints is not None:
            lin_x = np.hstack((lin_x, lin_waypoints[:, 0]))
            lin_y = np.hstack((lin_y, lin_waypoints[:, 1]))
            lin_z = np.hstack((lin_z, lin_waypoints[:, 2]))

        lin_x = np.hstack((lin_x, end_point[0]))
        lin_y = np.hstack((lin_y, end_point[1]))
        lin_z = np.hstack((lin_z, end_point[2]))

        start_quat = Rotation.from_quat(
            [
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w,
            ]
        )

        end_quat = Rotation.from_quat(
            [
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ]
        )

        # concatenate the waypoint rotations
        rs_waypoint = None
        if rot_waypoints is not None:
            for rpy in rot_waypoints:
                r = Rotation.from_euler("xyz", rpy, degrees=False)
                if rs_waypoint is None:
                    rs_waypoint = r
                else:
                    rs_waypoint = Rotation.concatenate([rs_waypoint, r])

        # concatenate the rotations
        if rs_waypoint is None:
            rs = Rotation.concatenate([start_quat, end_quat])
        else:
            rs = Rotation.concatenate([start_quat, rs_waypoint, end_quat])

        # create spline interpolator
        ss_waypoints = np.linspace(0, 1, 2)
        waypoints = np.vstack((lin_x, lin_y, lin_z)).T
        linear_spline = ta.SplineInterpolator(ss_waypoints, waypoints)

        # create rotation interpolator
        rotation_spline = Slerp([0, 1], rs)

        t = np.linspace(0, 1, self.num_points)

        # evaluate the spline
        lin_points = linear_spline.eval(t)
        rot_quats = rotation_spline(t).as_quat()

        return lin_points, rot_quats


if __name__ == "__main__":
    rospy.init_node("traj_utils", anonymous=True)

    # create a trajectory utils object
    traj_utils = TrajectoryUtils()

    # create a current pose
    current_pose = PoseStamped()
    current_pose.pose.position.x = 0.0
    current_pose.pose.position.y = 0.0
    current_pose.pose.position.z = 0.0

    current_quat = Rotation.from_euler(
        "xyz", [0, 0, 0], degrees=False
    ).as_quat()

    current_pose.pose.orientation.x = current_quat[0]
    current_pose.pose.orientation.y = current_quat[1]
    current_pose.pose.orientation.z = current_quat[2]
    current_pose.pose.orientation.w = current_quat[3]

    # create a target pose
    target_pose = PoseStamped()
    target_pose.pose.position.x = 1.0
    target_pose.pose.position.y = 1.0
    target_pose.pose.position.z = 1.0

    target_quat = Rotation.from_euler(
        "xyz", [0, 0, np.pi / 2], degrees=False
    ).as_quat()

    target_pose.pose.orientation.x = target_quat[0]
    target_pose.pose.orientation.y = target_quat[1]
    target_pose.pose.orientation.z = target_quat[2]
    target_pose.pose.orientation.w = target_quat[3]

    # compute the trajectory
    lin_points, rot_points = traj_utils.compute_trajectory(
        current_pose, target_pose
    )

    # publish the trajectory
    traj_utils.publish_pose_array(lin_points, rot_points)