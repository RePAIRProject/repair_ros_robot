#!/usr/bin/env python3

# this version works on python3 ubuntu 20
import rospy

from std_msgs.msg import Float64
from ec_msgs.msg import HandCmd

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from qb_device_srvs.srv import GetMeasurements

FREQ = 200


class QbHand:

    def __init__(self, side= "right", gazebo=False):
        self.side = side
        self.gazebo = gazebo

        if gazebo:
            self.gripperMsg = Float64()
            self.open_value = 0.0
            self.close_value = 0.9
        else:
            self.gripperMsg = HandCmd()
            self.open_value = 0.0
            self.close_value =18000.0
        
        if self.side == "right":
            qbhand_topic = "/qbhand1/control/qbhand1_synergy_trajectory_controller/command"
        elif self.side == "left":
            qbhand_topic = "/qbhand2/control/qbhand2_synergy_trajectory_controller/command"

        self.qb_hand_pub = rospy.Publisher(qbhand_topic, JointTrajectory, queue_size=10)

        rospy.sleep(1.0)

    def init_ros(self):
        
        
        # Simulation topic
        # hand_topic = "/"+side+"_hand_v1_wide/synergy_command"

        #rostopic pub /xbotcore/left_hand/command ec_msgs/HandCmd "{pos_ref: 0.0, pos_ref_2: 0.0, pos_ref_3: 0.0, vel_ref: 0.0, tor_ref: 0.0}"
        # 19000 close

        if(self.gazebo):
            sh_version = str(rospy.get_param("/sh_version"))
            print(sh_version)

            try:
                rospy.init_node("Qb_hand_"+self.side, anonymous=True)
            except rospy.exceptions.ROSException as e:
                var = 0  # print("Node has already been initialized, do nothing")

            self.rate = rospy.Rate(FREQ)
            #QUIRINO: fixed hand open/closure for mixed_hands
            if sh_version == "mixed_hands":
                if self.side == "right":
                    self.GripperPub = rospy.Publisher("/"+self.side+"_hand_v1_2_research/synergy_command", Float64, queue_size=3)
                elif self.side == "left":
                        self.GripperPub = rospy.Publisher("/"+self.side+"_hand_v1_wide/synergy_command", Float64, queue_size=3)
            else:
                self.GripperPub = rospy.Publisher("/"+self.side+"_hand_"+sh_version+"/synergy_command", Float64, queue_size=3)
        else:
            # hand_topic = "/xbotcore/"+self.side+"_hand/command"
            # self.GripperPub = rospy.Publisher(hand_topic, HandCmd, queue_size=3)
            pass
    
    def move_hand(self, aperture, secs=1.0):
        # moving
        #print('Moving qb Soft Hand..')
        if self.gazebo:
            self.gripperMsg.data = aperture
            self.GripperPub.publish(self.gripperMsg)

        elif self.gazebo == False:
            self.qbhand_control(aperture)

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
    
    def qbhand_control(self, val):
        
        if self.side == "right":
            qbhand_joints = 'qbhand1_synergy_joint'
        elif self.side == "left":
            qbhand_joints = 'qbhand2_synergy_joint'

        msg = JointTrajectory()
        msg.joint_names = [qbhand_joints]
        msg.header.stamp = rospy.Time.now()
        point = JointTrajectoryPoint()
        point.positions = [val]
        point.time_from_start = rospy.Duration(1)
        msg.points.append(point)

        self.qb_hand_pub.publish(msg)
        rospy.sleep(2)
    
    def read_current(self):
        if self.side == "right":
            num = 1
        elif self.side == "left":
            num = 2

        current_measure = "/qbhand" + str(num) + "/get_async_measurements"
        rospy.wait_for_service(current_measure)
        try:
            service = rospy.ServiceProxy(current_measure, GetMeasurements)
            request = GetMeasurements._request_class()
            request.id = num
            request.max_repeats = 0
            request.get_positions = False
            request.get_currents = True
            request.get_distinct_packages = False
            request.get_commands = False
            response = service(request)
            return response
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None

