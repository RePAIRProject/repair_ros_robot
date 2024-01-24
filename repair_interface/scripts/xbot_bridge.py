#!/usr/bin/env python3

# Importing necessary libraries
import rospy
from xbot_msgs.msg import JointCommand, JointState
from sensor_msgs.msg import JointState as JointStateMsg

# XBotClient class definition
class XBotClient:
    def __init__(self, node_handle):
        self.nh = node_handle

        # Publishers and subscribers for XBot
        self.xbot_state_sub = rospy.Subscriber(
            "/xbotcore/joint_states", JointState, self.xbot_state_callback
        )

        self.state_pub = rospy.Publisher("/joint_states", JointStateMsg, queue_size=10)

        self.nh.loginfo("XBot client initialized")

    def xbot_state_callback(self, msg: JointState):
        #print("XBot state callback")
        print(msg.header.stamp.to_sec())
        print(rospy.Time.now().to_sec())
        print(":::::::::::")
        # Process and handle XBot state information
        joint_state_msg = JointStateMsg()
        joint_state_msg.header.stamp = msg.header.stamp
        joint_state_msg.name = msg.name
        joint_state_msg.position = msg.motor_position
        joint_state_msg.velocity = msg.motor_velocity

        self.state_pub.publish(joint_state_msg)


if __name__ == "__main__":
    rospy.init_node("xbot_client_node")

    xbot_client = XBotClient(rospy)

    rospy.spin()
