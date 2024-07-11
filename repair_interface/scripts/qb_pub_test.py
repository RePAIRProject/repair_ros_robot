#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


    
def qbhand_contol():
    topic = "/qbhand1/control/qbhand1_synergy_trajectory_controller/command"

    pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)

    rospy.sleep(2)
    

    msg = JointTrajectory()
    msg.joint_names = ['qbhand1_synergy_joint']
    msg.header.stamp = rospy.Time.now()
    point = JointTrajectoryPoint()
    point.positions = [0.0]
    point.time_from_start = rospy.Duration(1)
    msg.points.append(point)

    pub.publish(msg)
    rospy.sleep(2)

    point.positions = [0.9]
    msg.header.stamp = rospy.Time.now()
    point.time_from_start = rospy.Duration(1)
    msg.points = []
    msg.points.append(point)
    pub.publish(msg)
    rospy.sleep(2)

    point.positions = [0.0]
    msg.header.stamp = rospy.Time.now()
    point.time_from_start = rospy.Duration(1)
    msg.points = []
    msg.points.append(point)
    pub.publish(msg)
    rospy.sleep(2)

if __name__ == "__main__":
    rospy.init_node("qbhand_control", anonymous=True)
    qbhand_contol()
    print('Finish!')