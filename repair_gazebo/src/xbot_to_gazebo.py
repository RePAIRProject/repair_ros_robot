#!/usr/bin/env python

import rospy
from xbot_msgs.msg import JointCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class XBotCommandRemapper:
    def __init__(self):
        # Publishers
        self.pub_torso = rospy.Publisher('/torso_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.pub_arm1 = rospy.Publisher("/arm_1_trajectory_controller/command", JointTrajectory, queue_size=1)
        self.pub_arm2 = rospy.Publisher("/arm_2_trajectory_controller/command", JointTrajectory, queue_size=1)
        
         # Joint name groups
        self.torso_joints = ['j_torso_1', 'j_torso_base']
        self.arm1_joints = ['j_arm_1_1', 'j_arm_1_2', 'j_arm_1_3', 'j_arm_1_4', 'j_arm_1_5', 'j_arm_1_6', 'j_arm_1_7']
        self.arm2_joints = ['j_arm_2_1', 'j_arm_2_2', 'j_arm_2_3', 'j_arm_2_4', 'j_arm_2_5', 'j_arm_2_6', 'j_arm_2_7']
        
        # Subscriber
        rospy.Subscriber('/xbotcore/command', JointCommand, self.callback)

        rospy.loginfo("XBotCommandRemapper subscribed to /xbotcore/command")

    def publish_trajectory(self, joint_names, joint_pos_dict, publisher):
        positions = [joint_pos_dict[j] for j in joint_names]
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time(0)
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(2.0)
        traj_msg.points.append(point)

        publisher.publish(traj_msg)

    def callback(self, msg):
        joint_pos_dict = dict(zip(msg.name, msg.position))

        # Check and publish if all Arm 1 joints are present
        if all(j in joint_pos_dict for j in self.arm1_joints):
            self.publish_trajectory(self.arm1_joints, joint_pos_dict, self.pub_arm1)
            rospy.loginfo_throttle(2.0, "Published Arm 1 trajectory")

        # Check and publish if all Arm 2 joints are present
        if all(j in joint_pos_dict for j in self.arm2_joints):
            self.publish_trajectory(self.arm2_joints, joint_pos_dict, self.pub_arm2)
            rospy.loginfo_throttle(2.0, "Published Arm 2 trajectory")

        # Check and publish if all torso joints are present
        if all(j in joint_pos_dict for j in self.torso_joints):
            self.publish_trajectory(self.torso_joints, joint_pos_dict, self.pub_torso)
            rospy.loginfo_throttle(2.0, "Published Torso trajectory")
            

if __name__ == '__main__':
    rospy.init_node('xbot_command_splitter')
    try:
        splitter = XBotCommandRemapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass