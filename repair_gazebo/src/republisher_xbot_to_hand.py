#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from ec_msgs.msg import HandCmd, HandStatus
from sensor_msgs.msg import JointState

class FloatToHandRemapper:
    def __init__(self):
        # Publishers
        #self.hand_cmd_pub = rospy.Publisher("/xbotcore/right_hand/command", HandCmd, queue_size=1)
        self.right_hand_command_pub = rospy.Publisher("/right_hand_v1_wide/synergy_command", Float64, queue_size=1)
        self.hand_status_pub = rospy.Publisher("/xbotcore/right_hand/status", HandStatus, queue_size=1)
        
        #self.hand_cmd_pub = rospy.Publisher("/xbotcore/right_hand/command", HandCmd, queue_size=1)
        self.left_hand_command_pub = rospy.Publisher("/left_hand_v1_wide/synergy_command", Float64, queue_size=1)
        self.hand_status_pub = rospy.Publisher("/xbotcore/left_hand/status", HandStatus, queue_size=1)

        # Subscribers (two separate callbacks)
        rospy.Subscriber("/xbotcore/right_hand/command", HandCmd, self.right_hand_cmd_callback)
        #rospy.Subscriber("/right_hand_v1_wide/synergy_command", Float64, self.float_to_cmd_callback)
        rospy.Subscriber("/right_hand_v1_wide/motor_state", JointState, self.right_hand_float_to_status_callback)
        
        rospy.Subscriber("/xbotcore/left_hand/command", HandCmd, self.left_hand_cmd_callback)
        #rospy.Subscriber("/left_hand_v1_wide/synergy_command", Float64, self.float_to_cmd_callback)
        rospy.Subscriber("/left_hand_v1_wide/motor_state", JointState, self.left_hand_float_to_status_callback)

    def right_hand_cmd_callback(self, msg):
        # Remap pos_ref (or you could use tor_ref or others as needed)
        float_value = float(msg.pos_ref)
        self.right_hand_command_pub.publish(Float64(data=float_value))
        
    def left_hand_cmd_callback(self, msg):
        # Remap pos_ref (or you could use tor_ref or others as needed)
        float_value = float(msg.pos_ref)
        self.left_hand_command_pub.publish(Float64(data=float_value))

    # def float_to_cmd_callback(self, msg):
    #     """Publishes Float64 input as a HandCmd message."""
    #     value = msg.data
    #     hand_cmd = HandCmd()
    #     hand_cmd.pos_ref = value
    #     hand_cmd.pos_ref_2 = 0.0
    #     hand_cmd.pos_ref_3 = 0.0
    #     hand_cmd.vel_ref = 0.0
    #     hand_cmd.tor_ref = 0.0
    #     self.hand_cmd_pub.publish(hand_cmd)

    def right_hand_float_to_status_callback(self, msg):
        """Convert JointState to HandStatus."""
        if not msg.position:
            return  # Don't proceed with empty joint positions
        hand_status = HandStatus()
        hand_status.hand_name = "right_hand_v1_wide"
        hand_status.motor_pos = msg.position[0] if len(msg.position) > 0 else 0.0
        hand_status.motor_pos_2 = 0.0
        hand_status.motor_pos_3 = 0.0
        hand_status.m1_an_1 = 0
        hand_status.m1_an_2 = 0
        hand_status.m1_an_3 = 0
        hand_status.m2_an_1 = 0
        hand_status.m2_an_2 = 0
        hand_status.m2_an_3 = 0
        hand_status.m3_an_1 = 0
        hand_status.m3_an_2 = 0
        hand_status.m1_curr = 0
        hand_status.m2_curr = 0
        hand_status.m3_curr = 0
        hand_status.fault = 0
        hand_status.curr = 0
        self.hand_status_pub.publish(hand_status)
        
    def left_hand_float_to_status_callback(self, msg):
        """Convert JointState to HandStatus."""
        if not msg.position:
            return  # Don't proceed with empty joint positions
        hand_status = HandStatus()
        hand_status.hand_name = "left_hand_v1_wide"
        hand_status.motor_pos = msg.position[0] if len(msg.position) > 0 else 0.0
        hand_status.motor_pos_2 = 0.0
        hand_status.motor_pos_3 = 0.0
        hand_status.m1_an_1 = 0
        hand_status.m1_an_2 = 0
        hand_status.m1_an_3 = 0
        hand_status.m2_an_1 = 0
        hand_status.m2_an_2 = 0
        hand_status.m2_an_3 = 0
        hand_status.m3_an_1 = 0
        hand_status.m3_an_2 = 0
        hand_status.m1_curr = 0
        hand_status.m2_curr = 0
        hand_status.m3_curr = 0
        hand_status.fault = 0
        hand_status.curr = 0
        self.hand_status_pub.publish(hand_status)

if __name__ == '__main__':
    rospy.init_node('float_to_hand_remap_node')
    FloatToHandRemapper()
    rospy.spin()