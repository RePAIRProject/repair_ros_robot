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

    hand_pub = rospy.Publisher('/xbotcore/right_hand/command', HandCmd, queue_size=10)
    
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
    
    
    # 1
    # translation: [   0.31, 0.03872, -0.8109]
    # rotation   : [ 0.7235, 0.08488,   0.677,  0.1054]

    # 2
    # translation: [ 0.2758, -0.1254, -0.8083]
    # rotation   : [0.7981, 0.1693, 0.5712,  -0.09]


    # obliquo
    # 1
    # translation: [ 0.3116,  0.1199, -0.8384]
    # rotation   : [ 0.6952, -0.3916,  0.5454,  0.2568]
    # 2
    # translation: [  0.1835, -0.04574,  -0.8058]
    # rotation   : [ 0.8107, -0.1026,  0.4771,  0.3236]

    
    # good solution
        # 1
    # translation: [ 0.3116,  0.1199, -0.6675]
    # rotation   : [ 0.7917, 0.09927,  0.2853,   0.531]
        # 2
    # translation: [-0.1241,  0.1048, 0.07112]
    # rotation   : [  0.1116, -0.03152,   0.9101,   0.3978]



    torso_start, _, _ = ci.getPoseReference('torso_1')
    arm_1_start, _, _ = ci.getPoseReference('arm_1_tcp')
    arm_2_start, _, _ = ci.getPoseReference('arm_2_tcp')

    print(waypoints_torso)
    print(torso_start)
    print(arm_1_start)
    print(arm_2_start)



    
    
    #SMALL AND WIDE HAND
    # add_wp(Affine3(pos=[0.3116,  0.1199, -0.8384], rot=[0.6952, -0.3916,  0.5454,  0.2568]), 2*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.1835, -0.04574,  -0.8058], rot=[0.8107, -0.1026,  0.4771,  0.3236]), 2*time, waypoints_arm_2)
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')

    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # #left hand rotation
    # # translation: [  0.2466, 0.009412,  -0.7748]
    # # rotation   : [-0.5472,  0.6069, -0.5726, 0.06626]


    # add_wp(Affine3(pos=[0.2466, 0.009412,  -0.7748], rot=[-0.5472,  0.6069, -0.5726, 0.06626]), 1*time, waypoints_arm_2)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    # ci.waitReachCompleted('arm_2_tcp')
    
    # waypoints_arm_2.clear()

    # exit(0)


    
    # #right hand rotation
    # # translation: [ 0.3488,  0.1087, -0.7303]
    # #rotation   : [ 0.7655, -0.2252,  0.4736,  0.3728]

    # add_wp(Affine3(pos=[0.3488,  0.1087, -0.7303], rot=[0.7655, -0.2252,  0.4736,  0.3728]), 1*time, waypoints_arm_1)
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)

    # ci.waitReachCompleted('arm_1_tcp')
    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # #right and left rotation
    # # translation: [ 0.3357, 0.09681, -0.7826]
    # # rotation   : [ 0.7122, 0.05171,  0.4219,  0.5587]
    # # translation: [  0.2419, 0.001909,  -0.7709]
    # # rotation   : [-0.4922,  0.6523, -0.5646,  0.1159]

    # add_wp(Affine3(pos=[0.3357, 0.09681, -0.7826], rot=[0.7122, 0.05171,  0.4219,  0.5587]), 1*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.2419, 0.001909,  -0.7709], rot=[-0.4922,  0.6523, -0.5646,  0.1159]), 1*time, waypoints_arm_2)
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')

    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()


    # #lift



    # print('Motion completed!')
    # exit(0)


    
    
    
    
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