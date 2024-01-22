#!/usr/bin/env python3

# this version works on python3 ubuntu 20
import rospy

from std_msgs.msg import Float64
from ec_msgs.msg import HandCmd

FREQ = 200


class QbHand:
    def __init__(self):
        self.gripperMsg = Float64()
        self.init_ros()
        self.init_params()

        rospy.sleep(1)

    def close_hand(self, aperture, secs=0.5):
        # close
        print('Closing qb Soft Hand..')
        pose = aperture
        # self.gripperMsg.pos_ref = pose
        self.gripperMsg = pose
        # print(self.gripperMsg)
        self.GripperPub.publish(self.gripperMsg)

        print('wait to finish')
        rospy.sleep(secs)

    def open_hand(self, secs=0.5):
        # open
        print('Opening..')
        pose = 0
        # self.gripperMsg.pos_ref = pose
        self.gripperMsg = pose
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
        # Simulation topic
        hand_topic = "/right_hand_v1s/synergy_command"
        self.GripperPub = rospy.Publisher(hand_topic,
                                          Float64, queue_size=3)
        #rostopic pub /xbotcore/left_hand/command ec_msgs/HandCmd "{pos_ref: 0.0, pos_ref_2: 0.0, pos_ref_3: 0.0, vel_ref: 0.0, tor_ref: 0.0}"
        # 19000 close

        # real tobot topic
        #hand_topic = "/xbotcore/right_hand/command"
        #self.GripperPub = rospy.Publisher(hand_topic,
        #                                  HandCmd, queue_size=3)

        
if __name__ == "__main__":
    hand = QbHand()
    value = 19000
    hand.close_hand(value)
    hand.open_hand()
    print('Finish!')