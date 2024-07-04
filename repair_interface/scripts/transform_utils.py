import rospy
import tf
import math

from geometry_msgs.msg import PoseStamped, Quaternion

from typing import List, Union


class TransformUtils:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.wait_for_transform = 5
        self.transform_tries = 5

    def transform_pose_with_retries(
        self,
        reference_pose: PoseStamped,
        target_frame: str,
        retries: int = 5,
        execute_arm: bool = False,
        offset: List[float] = [0.0, 0.0, 0.0],
    ) -> Union[PoseStamped, None]:
        """Transform pose with multiple retries

        input reference_pose: The reference pose.
        input target_frame: The name of the taget frame.
        input retries: The number of retries.
        input execute_arm: If true, the pose will be rotated by 180 degrees around the x axis.
        input offset: [r, p, y] offset in rad to be added to the pose if execute_arm is true.

        :return: The updated state.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        transformed_pose = None
        for i in range(0, retries):
            transformed_pose = self.transform_pose(reference_pose, target_frame)
            if transformed_pose:
                break

        if execute_arm:
            # rotate around z axis by 90 degrees
            euler = tf.transformations.euler_from_quaternion(
                [
                    transformed_pose.pose.orientation.x,
                    transformed_pose.pose.orientation.y,
                    transformed_pose.pose.orientation.z,
                    transformed_pose.pose.orientation.w,
                ]
            )
            q = tf.transformations.quaternion_from_euler(
                math.pi + offset[0], offset[1] + euler[1], euler[2] + offset[2]
            )
            transformed_pose.pose.orientation = Quaternion(*q)

        return transformed_pose

    def transform_pose(
        self, reference_pose: PoseStamped, target_frame: str
    ) -> Union[PoseStamped, None]:
        """
        Transforms a given pose into the target frame.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_frame: The name of the taget frame.
        :type target_frame: String

        :return: The pose in the target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            # common_time = self.listener.getLatestCommonTime(
            #     target_frame, reference_pose.header.frame_id
            # )

            self.listener.waitForTransform(
                target_frame,
                reference_pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(self.wait_for_transform),
            )
            # reference_pose.header.stamp = common_time

            transformed_pose = self.listener.transformPose(target_frame, reference_pose)

            return transformed_pose

        except tf.Exception as error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None
