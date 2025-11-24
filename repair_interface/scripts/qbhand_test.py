#!/usr/bin/env python3

# this version works on python3 ubuntu 20
import rospy

from std_msgs.msg import Float64
from ec_msgs.msg import HandCmd, HandStatus

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

FREQ = 200

class QbHand:
    def __init__(self, side= "right", gazebo=False):
        self.side = side
        self.gazebo = gazebo
        if gazebo:
            self.gripperMsg = HandCmd()
            self.open_value = 0.0
            self.close_value = 0.9
            self.hand_status = None
        else:
            if side == "left":
                self.gripperMsg = HandCmd()
                self.open_value = 0.0
                self.close_value =14000.0
                self.hand_status = None
            else:
                self.gripperMsg = HandCmd()
                self.open_value = 0.0
                self.close_value =20000.0
                self.hand_status = None

        self.init_params()

        # if self.side == "right":
        #     # topic = "/qbhand1/control/qbhand1_synergy_trajectory_controller/command"
        #     topic = "/xbotcore/"+self.side+"_hand/command"
        #     self.qb_hand_pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)
        # elif self.side == "left":
        topic = "/xbotcore/"+self.side+"_hand/command"
        #topic = f"/{self.side}_hand_v1_wide/synergy_command"
        self.GripperPub = rospy.Publisher(topic, HandCmd, queue_size=3)

        topic = "/xbotcore/"+self.side+"_hand/status"
        #topic = f"/{self.side}_hand_v1_wide/motor_state"
        rospy.Subscriber(topic, HandStatus, self.hand_current_callback)

        rospy.sleep(2)
        
    def hand_current_callback(self, msg):
        self.hand_status = msg
        
    def get_current(self):
        return self.hand_status

    def move_hand(self, aperture, secs=1.0):
        # moving
        if self.gazebo:
            self.gripperMsg.pos_ref = aperture
            self.GripperPub.publish(self.gripperMsg)

        elif self.gazebo == False and self.side == "right":
            self.gripperMsg.pos_ref = aperture
            self.GripperPub.publish(self.gripperMsg)
        
        elif self.gazebo == False and self.side == "left":
            #QUIRINO, TO_CHECK
            self.gripperMsg.pos_ref = aperture
            self.GripperPub.publish(self.gripperMsg)
        
        # print('wait to finish')
        rospy.sleep(secs)
        
    def close_hand(self):
        # close
        # print('Closing qb Soft Hand..')
        self.move_hand(self.close_value)

    def close_hand_2(self, used_hand, gazebo_flag=False):
        # close
        if used_hand == "right":
            print('Closing QbHand..')
            if(gazebo_flag == True):self.move_hand(0.9)
            else:self.move_hand(11500)
        else:
            print('Closing Wide Hand..')
            if(gazebo_flag == True):self.move_hand(0.9)
            else:self.move_hand(15500)


    def open_hand(self, secs=0.5):
        # open
        # print("Opening qb Soft Hand..")
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
            if side == "left":
                hand_topic = "/xbotcore/"+self.side+"_hand/command"
                # self.GripperPub = rospy.Publisher(hand_topic, HandCmd, queue_size=3)
            else:
                pass
        
        
    def qbhand_contol(self, val):
        msg = JointTrajectory()
        msg.joint_names = ['qbhand1_synergy_joint']
        msg.header.stamp = rospy.Time.now()
        point = JointTrajectoryPoint()
        point.positions = [val]
        point.time_from_start = rospy.Duration(1)
        msg.points.append(point)

        # self.qb_hand_pub.publish(msg)
        self.GripperPub.publish(msg)
        rospy.sleep(2)


        
if __name__ == "__main__":
    rospy.init_node('qbhand_test', anonymous=True)
    gazebo = False
    side = "right"
    hand_api = QbHand(side, gazebo)

    if hand_api.gazebo:
        value = 0.5
    else:
        value = 19000.0/2
    #hand_api.move_hand(value)
    #hand_api.close_hand()
    #hand_api.open_hand()

    ### 4. close hand
    hand_api.close_hand()
    print('Closing!')

    qbhand_curr = hand_api.get_current()

    print('Is the fresco present?')
    while (not (int(qbhand_curr.m1_curr) > 100 and int(qbhand_curr.m2_curr) > 100)):

        hand_api.open_hand()
        rospy.sleep(1)

        hand_api.close_hand()  
        rospy.sleep(1)
        qbhand_curr = hand_api.get_current()

    print('Yes')

    print('Finish!')