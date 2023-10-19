#!/usr/bin/env python3

# this version works on python3 ubuntu 20
import rospy
from std_msgs.msg import Float64

FREQ = 200


class QbHand:
    def __init__(self):
        self.gripperMsg = Float64()
        self.init_ros()
        self.init_params()

        rospy.sleep(1)

    def close_hand(self, aperture, secs=1):
        # close
        print('Closing qb Soft Hand..')
        self.gripper_open = aperture
        self.gripperMsg.data = self.gripper_open
        # print(self.gripperMsg)
        self.GripperPub.publish(self.gripperMsg)

        print('wait to finish')
        rospy.sleep(secs)

    def open_hand(self, secs=1):
        # open
        print('Opening..')
        self.gripper_open = 0
        self.gripperMsg.data = self.gripper_open
        # print(self.gripperMsg)
        self.GripperPub.publish(self.gripperMsg)

        print('wait to finish')
        rospy.sleep(secs)

    def init_params(self):
        pass

    def init_ros(self):
        try:
            rospy.init_node('Qb_hand', anonymous=True)
        except rospy.exceptions.ROSException as e:
            var = 0  # print("Node has already been initialized, do nothing")

        self.rate = rospy.Rate(FREQ)
        self.GripperPub = rospy.Publisher("/left_hand_v1s/synergy_command",
                                          Float64, queue_size=3)

if __name__ == "__main__":
    hand = QbHand()
    value = 0.7
    hand.close_hand(value)
    hand.open_hand()
    print('Finish!')