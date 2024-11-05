#!/usr/bin/env python3

from qbhand_test import QbHand
import rospy

if __name__ == '__main__':

    rospy.init_node('qbhand_wide', anonymous=True)
    gazebo = False
    side = "left"
    hand_api = QbHand(side, gazebo)

    hand_api.open_hand()
    rospy.sleep(2)
    hand_api.close_hand()
    rospy.sleep(2)
    hand_api.open_hand()