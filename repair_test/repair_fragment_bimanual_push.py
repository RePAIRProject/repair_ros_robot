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
    # translation: [   -0.3, -0.2711,    1.97]
    # rotation   : [ 1.868e-08, -2.451e-08,    -0.2724,     0.9622]

    add_wp(Affine3(pos=[-0.3, -0.2711,    1.97], rot=[ 1.868e-08, -2.451e-08,    -0.2724,     0.9622]), 2*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')

    waypoints_torso.clear()

    #HANDS ON SAND
    # translation: [ 0.2176,  0.1078, -0.8282]
    # rotation   : [  0.7593, -0.08374,    0.645,  0.02138]
    # translation: [ 0.1994, -0.0531,  -0.804]
    # rotation   : [  0.7617,  -0.1002,   0.6346, -0.08414]

    # translation: [ 0.2176,  0.1078, -0.8282]
    # rotation   : [  0.7593, -0.08374,    0.645,  0.02138]
    # translation: [  0.2101, -0.08812,  -0.8363]
    # rotation   : [   0.719, 0.008977,   0.6687,   -0.189]

    # translation: [ 0.2471,  0.1196, -0.8233]
    # rotation   : [  0.7239, -0.04617,   0.6522,   0.2201]
    # translation: [ 0.2078, -0.0715, -0.7799]
    # rotation   : [ 0.8281, 0.04815,  0.5567, 0.04421]

    # ver 2 

#     translation: [   -0.3, -0.2711,    1.97]
# rotation   : [ 1.868e-08, -2.451e-08,    -0.2724,     0.9622]
# translation: [ 0.2471,  0.1196, -0.8233]
# rotation   : [  0.7239, -0.04617,   0.6522,   0.2201]
# translation: [  0.1957, -0.08175,  -0.8081]
# rotation   : [  0.8321, -0.06132,    0.551,  0.01696]

 # translation: [   -0.3, -0.2711,    1.97]
# rotation   : [ 1.868e-08, -2.451e-08,    -0.2724,     0.9622]
# translation: [ 0.2471,  0.1196, -0.8233]
# rotation   : [  0.7239, -0.04617,   0.6522,   0.2201]
# translation: [  0.1957, -0.08175,  -0.8081]
# rotation   : [ 0.7958, -0.2508,  0.5401,   -0.11]


    add_wp(Affine3(pos=[0.2471,  0.1196, -0.8233], rot=[0.7239, -0.04617,   0.6522,   0.2201]), 2*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.1957, -0.08175,  -0.8081], rot=[0.8321, -0.06132,    0.551,  0.01696]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    # rotation of right hand
    add_wp(Affine3(pos=[0.1957, -0.08175,  -0.8081], rot=[ 0.7958, -0.2508,  0.5401,   -0.11]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_2_tcp')
    waypoints_arm_2.clear()

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


    add_wp(Affine3(pos=[0.2471,  0.1196, -0.6691], rot=[0.7239, -0.04614,   0.6522,   0.2201]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)

    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()


    exit(0)
    #TWO SMALL HANDS

    #AVVICINAMENTO
    # translation: [ 0.3586,  0.2586, -0.8023]
    # rotation   : [ 0.7511, -0.2193,    0.47,  0.4085]
    # translation: [  0.1389, 0.009446,  -0.8135]
    # rotation   : [ 0.7137, -0.4494,   0.516, -0.1496]


    add_wp(Affine3(pos=[0.3586,  0.2586, -0.8023], rot=[0.7511, -0.2193,    0.47,  0.4085]), 2*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.1389, 0.009446,  -0.8135], rot=[0.7137, -0.4494,   0.516, -0.1496]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()



    #LEFT ROTATION
    # translation: [ 0.3586,  0.2586, -0.8023]
    # rotation   : [0.7707, 0.1349, 0.2413, 0.5741]
    # translation: [  0.1389, 0.009446,  -0.8135]
    # rotation   : [ 0.7137, -0.4494,   0.516, -0.1496]

    add_wp(Affine3(pos=[0.3586,  0.2586, -0.8023], rot=[0.7707, 0.1349, 0.2413, 0.5741]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)

    ci.waitReachCompleted('arm_1_tcp')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    #RIGHT ROTATION
    # translation: [ 0.3586,  0.2586, -0.8023]
    # rotation   : [0.7707, 0.1349, 0.2413, 0.5741]
    # translation: [ 0.1414, 0.02862, -0.8197]
    # rotation   : [ 0.6157, -0.5764,  0.4784, -0.2446]

    # add_wp(Affine3(pos=[0.3586,  0.2586, -0.8023], rot=[0.7707, 0.1349, 0.2413, 0.5741]), 2*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.1414, 0.02862, -0.8197], rot=[0.6157, -0.5764,  0.4784, -0.2446]), 2*time, waypoints_arm_2)
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')

    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()
    i_loop = 0
    msg = HandCmd()
    msg.pos_ref = 7500.
    while i_loop < 5:
        hand_pub.publish(msg)
        i_loop += 1
        rate.sleep()



    ci.getTask('arm_2_tcp').setBaseLink('arm_1_tcp')


    #LIFT
    # translation: [ 0.3586,  0.2586, -0.5659]
    # rotation   : [0.7707, 0.1349, 0.2413, 0.5741]
    # translation: [-0.3094, 0.08039, 0.09101]
    # rotation   : [  0.347, -0.0123,   0.775,  0.5281]
    add_wp(Affine3(pos=[0.3586,  0.2586, -0.6], rot=[0.7707, 0.1349, 0.2413, 0.5741]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)

    ci.waitReachCompleted('arm_1_tcp')

    waypoints_arm_1.clear()



    
    
    #TORSO ROTATION AND TRANSLATION

    # translation: [-0.0706,  0.5556,    1.97]
    # rotation   : [-1.718e-08, -2.476e-08,      0.556,     0.8312]
    add_wp(Affine3(pos=[-0.0706,  0.5556,    1.97], rot=[ 0-1.718e-08, -2.476e-08,      0.556,     0.8312]), 2*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')

    waypoints_torso.clear()



    # PUT DOWN

    #     translation: [ 0.3586, 0.06366,   -0.85]
    # rotation   : [0.7707, 0.1349, 0.2413, 0.5741]

    add_wp(Affine3(pos=[ 0.3586, 0.06366,   -0.8], rot=[0.7707, 0.1349, 0.2413, 0.5741]), 2*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')

    waypoints_arm_1.clear()


    # RELEASE
    #RIGHT
    # translation: [0.02314,  0.5567,   1.146]
    # rotation   : [ 0.8323, -0.1365,  0.2617, -0.4692]

    ci.getTask('arm_2_tcp').setBaseLink('world')
    
    add_wp(Affine3(pos=[0.02314,  0.5567,   1.146], rot=[ 0.8323, -0.1365,  0.2617, -0.4692]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_arm_2.clear()

    i_loop = 0
    msg = HandCmd()
    msg.pos_ref = 0.
    while i_loop < 5:
        hand_pub.publish(msg)
        i_loop += 1
        rate.sleep()

    #LEFT
    # translation: [ 0.3215,  0.1443, -0.8353]
    # rotation   : [0.7707, 0.1349, 0.2413, 0.5741]


    add_wp(Affine3(pos=[ 0.3215,  0.1443, -0.8353], rot=[0.7707, 0.1349, 0.2413, 0.5741]), 1*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')

    waypoints_arm_1.clear()

    # AFTER
    # translation: [ 0.3716,  0.1464, -0.7253]
    # rotation   : [0.7707, 0.1349, 0.2413, 0.5741]
    # translation: [0.1352, 0.5332,  1.255]
    # rotation   : [ 0.8323, -0.1365,  0.2617, -0.4692]

    add_wp(Affine3(pos=[0.3716,  0.1464, -0.7253], rot=[0.7707, 0.1349, 0.2413, 0.5741]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.1352, 0.5332,  1.255], rot=[0.8323, -0.1365,  0.2617, -0.4692]), 1*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

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