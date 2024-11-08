#!/usr/bin/env python

import xbot_interface.config_options as co
import xbot_interface.xbot_interface as xb
from cartesian_interface.pyci_all import *
import numpy as np
import rospy

from ec_msgs.msg import HandCmd

def add_wp(mat, time, wp_list):

    wp = pyci.WayPoint(mat, time)
    wp_list.append(wp)

def main():
    rospy.init_node('repair_box_pick')

    right_hand_pub = rospy.Publisher('/xbotcore/right_hand/command', HandCmd, queue_size=10)
    left_hand_pub = rospy.Publisher('/xbotcore/left_hand/command', HandCmd, queue_size=10)

    # Set the rate at which to publish messages (10 Hz)
    rate = rospy.Rate(10)
    time = 3.0

    i_loop = 0
    msg = HandCmd()
    msg.pos_ref = 0.
    while i_loop < 5:
        right_hand_pub.publish(msg)
        left_hand_pub.publish(msg)
        i_loop += 1
        rate.sleep()

    # robot = get_robot()
    # Rise arms and send them to the back
    # q0 = robot.getMotorPosition()

    ci = pyci.CartesianInterfaceRos()
    ci.update()

    waypoints_arm_1 = []
    waypoints_arm_2 = []
    waypoints_torso = []
    


    torso_start, _, _ = ci.getPoseReference('torso_1')
    arm_1_start, _, _ = ci.getPoseReference('arm_1_tcp')
    arm_2_start, _, _ = ci.getPoseReference('arm_2_tcp')

    print(torso_start)
    print(arm_1_start)
    print(arm_2_start)
    # exit(0)

    #TORSO ROTATION AND TRANSLATION
    add_wp(Affine3(pos=[-0.3, -0.2711,    1.97], rot=[ 1.868e-08, -2.451e-08,    -0.2724,     0.9622]), 2*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')

    waypoints_torso.clear()


    add_wp(Affine3(pos=[0.2471,  0.1196, -0.8233], rot=[0.7239, -0.04617,   0.6522,   0.2201]), 2*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.1957, -0.08175,  -0.8081], rot=[0.8321, -0.06132,    0.551,  0.01696]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    # rotation of right hand
    add_wp(Affine3(pos=[0.1957, -0.07519,  -0.7919], rot=[0.8058,  -0.228,  0.5037, -0.2124]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_2_tcp')
    waypoints_arm_2.clear()


    add_wp(Affine3(pos=[0.2417,  0.0977, -0.8431], rot=[0.7038, -0.02568,   0.6707,   0.2328]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)

    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()


    i_loop = 0
    msg = HandCmd()
    msg.pos_ref = 7500.
    while i_loop < 5:
        right_hand_pub.publish(msg)
        left_hand_pub.publish(msg)
        i_loop += 1
        rate.sleep()


    # raise hands
    ci.getTask('arm_2_tcp').setBaseLink('arm_1_tcp')


    add_wp(Affine3(pos=[ 0.2417,  0.0977, -0.7], rot=[0.7038, -0.02568,   0.6707,   0.2328]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)

    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()



    

    #TORSO ROTATION AND TRANSLATION

    # translation: [-0.3457,  0.3182,    1.97]
    #  rotation   : [-1.481e-08, -7.179e-08,     0.1569,     0.9876]

    add_wp(Affine3(pos=[-0.3457,  0.3182,    1.97], rot=[-1.481e-08, -7.179e-08,     0.1569,     0.9876]), 2*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')

    waypoints_torso.clear()




    # PUT DOWN

    # translation: [ 0.2417,  0.0977, -0.8137]
    # rotation   : [  0.7038, -0.02568,   0.6707,   0.2328]


    add_wp(Affine3(pos=[0.2417,  0.0977, -0.8137], rot=[ 0.7038, -0.02568,   0.6707,   0.2328]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')

    waypoints_arm_1.clear()



    # RELEASE

    ci.getTask('arm_2_tcp').setBaseLink('torso_1')
    


    add_wp(Affine3(pos=[0.2417,  0.1577, -0.7541], rot=[ 0.7038, -0.02568,   0.6707,   0.2328]), 2*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.1957, -0.1976, -0.7485], rot=[ 0.7986, -0.2456,  0.5362,   -0.12]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    i_loop = 0
    msg = HandCmd()
    msg.pos_ref = 0.
    while i_loop < 5:
        right_hand_pub.publish(msg)
        left_hand_pub.publish(msg)
        i_loop += 1
        rate.sleep()


    #prepare to push 1
    add_wp(Affine3(pos=[0.2997,  0.3058, -0.7691], rot=[0.6776, -0.009794,    0.6952,    0.2397]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()

    # move other hand
    add_wp(Affine3(pos=[ 0.2236,  -0.225, -0.6323], rot=[0.7986, -0.2456,  0.5362,   -0.12]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    waypoints_arm_2.clear()


    #prepare to push 2
    add_wp(Affine3(pos=[0.2997,  0.3058, -0.8265], rot=[0.6776, -0.009794,    0.6952,    0.2397]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()

    #prepare to push 3
    add_wp(Affine3(pos=[0.268,  0.1672, -0.8337], rot=[0.6651, -0.07354,   0.6782,   0.3037]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()

    # raise hand
    add_wp(Affine3(pos=[0.268,  0.1672, -0.7175], rot=[0.6651, -0.07354,   0.6782,   0.3037]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()

    # return to grab fragment
    add_wp(Affine3(pos=[-0.3, -0.5156,    1.97], rot=[5.213e-09, -2.887e-08,    -0.1002,      0.995]), 2*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')
    waypoints_torso.clear()

    # move hand 1 out of the way
    add_wp(Affine3(pos=[0.268,  0.3312, -0.7309], rot=[0.6651, -0.07354,   0.6782,   0.3037]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    waypoints_arm_1.clear()

    # move right hand to fragment before grabbing

    # translation: [  0.347, -0.2963, -0.8085]
    # rotation   : [0.4706,  0.577, 0.4622, 0.4816]

    add_wp(Affine3(pos=[ 0.3838, -0.2804, -0.8075], rot=[0.4706,  0.577, 0.4622, 0.4816]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_2_tcp')
    waypoints_arm_2.clear()

    i_loop = 0
    msg = HandCmd()
    msg.pos_ref = 19000.
    while i_loop < 5:
        right_hand_pub.publish(msg)
        i_loop += 1
        rate.sleep()

    # raise hand
    add_wp(Affine3(pos=[ 0.3838, -0.2804, -0.7075], rot=[0.4706,  0.577, 0.4622, 0.4816]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_2_tcp')
    waypoints_arm_2.clear()

    # move torso and hand to drop fragment
    # translation: [  -0.3, 0.5465,   1.97]
    # rotation   : [  -1.3e-08, -2.772e-08,     0.1559,     0.9878]

    add_wp(Affine3(pos=[-0.3, 0.5465,   1.97], rot=[-1.3e-08, -2.772e-08,     0.1559,     0.9878]), 4*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')
    
    waypoints_torso.clear()
    

    # translation: [ 0.2895, -0.2762, -0.7852]
    # rotation   : [0.4706,  0.577, 0.4622, 0.4816]

    add_wp(Affine3(pos=[ 0.2895, -0.2762, -0.7852], rot=[0.4706,  0.577, 0.4622, 0.4816]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_2_tcp')
    waypoints_arm_2.clear()

    i_loop = 0
    msg = HandCmd()
    msg.pos_ref = 0.
    while i_loop < 5:
        right_hand_pub.publish(msg)
        i_loop += 1
        rate.sleep()

    #START POINT

    ci.getTask('arm_2_tcp').setBaseLink('torso_1')

    add_wp(Affine3(pos=arm_1_start.translation, rot=arm_1_start.quaternion), 2*time, waypoints_arm_1)
    add_wp(Affine3(pos=arm_2_start.translation, rot=arm_2_start.quaternion), 2*time, waypoints_arm_2)
    add_wp(Affine3(pos=torso_start.translation, rot=torso_start.quaternion), 2*time, waypoints_torso)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.setWaypoints('torso_1', waypoints_torso)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')
    ci.waitReachCompleted('torso_1')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()
    waypoints_torso.clear()


    print('Motion completed!')
    exit(0)











if __name__ == '__main__':
    main()