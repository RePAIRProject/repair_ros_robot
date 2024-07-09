#!/usr/bin/env python3

# this version works on python3 ubuntu 20
import rospy

from std_msgs.msg import Float64
from ec_msgs.msg import HandCmd
from ec_msgs.srv import GetMeasurements

FREQ = 200


class QbHand:
    def __init__(self, side= "right", gazebo=False):
        self.side = side
        self.gazebo = gazebo
        if gazebo:
            self.gripperMsg = Float64()
            self.open_value = 0.0
            self.close_value = 1.0
        else:
            self.gripperMsg = HandCmd()
            self.open_value = 0.0
            self.close_value =19000.0

        self.init_ros()
        self.init_params()
        rospy.sleep(1.0)
        

    def move_hand(self, aperture, secs=1.0):
        # moving
        #print('Moving qb Soft Hand..')
        if self.gazebo:
            self.gripperMsg.data = aperture
        else:
            self.gripperMsg.pos_ref = aperture
        # print(self.gripperMsg)
        self.GripperPub.publish(self.gripperMsg)

        print('wait to finish')
        rospy.sleep(secs)
        
    def close_hand(self):
        # close
        print('Closing qb Soft Hand..')
        self.move_hand(self.close_value)

    def open_hand(self, secs=0.5):
        # open
        print("Opening qb Soft Hand..")
        self.move_hand(self.open_value)

    def init_params(self):
        pass

    def init_ros(self):
        sh_version = str(rospy.get_param("/sh_version"))
        print(sh_version)
        try:
            rospy.init_node("Qb_hand_"+self.side, anonymous=True)
        except rospy.exceptions.ROSException as e:
            var = 0  # print("Node has already been initialized, do nothing")

        self.rate = rospy.Rate(FREQ)
        # Simulation topic
        # hand_topic = "/"+side+"_hand_v1_wide/synergy_command"

        #rostopic pub /xbotcore/left_hand/command ec_msgs/HandCmd "{pos_ref: 0.0, pos_ref_2: 0.0, pos_ref_3: 0.0, vel_ref: 0.0, tor_ref: 0.0}"
        # 19000 close

        if(self.gazebo):
            #QUIRINO: fixed hand open/closure for mixed_hands
            if sh_version == "mixed_hands":
                if self.side == "right":
                    self.GripperPub = rospy.Publisher("/"+self.side+"_hand_v1_2_research/synergy_command", Float64, queue_size=3)
                elif self.side == "left":
                        self.GripperPub = rospy.Publisher("/"+self.side+"_hand_v1_wide/synergy_command", Float64, queue_size=3)
            else:
                self.GripperPub = rospy.Publisher("/"+self.side+"_hand_"+sh_version+"/synergy_command", Float64, queue_size=3)
        else:
            hand_topic = "/xbotcore/"+self.side+"_hand/command"
            self.GripperPub = rospy.Publisher(hand_topic, HandCmd, queue_size=3)

    def read_current(self, side, sh_version):
        # current_measure = "/qbhand1/get_async_measurements"
        current_measure = "/" + side + "_hand_" + sh_version + "/get_async_measurments"
        rospy.wait_for_service(current_measure)
        try:
            service = rospy.ServiceProxy(current_measure, GetMeasurements)
            request = GetMeasurements.Request()
            request.id = 1
            request.max_repeats = 0
            request.get_positions = False
            request.get_currents = True
            request.get_distinct_packages = False
            request.get_commands = False
            response = service(request)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
if __name__ == "__main__":
    gazebo = False
    side = "right"
    hand_api = QbHand(side, gazebo)

    if hand_api.gazebo:
        value = 0.5
    else:
        value = 19000.0/2
    #hand_api.move_hand(value)
    hand_api.close_hand()
    hand_api.open_hand()
    print('Finish!')
