#!/usr/bin/env python

import xbot_interface.config_options as co
import xbot_interface.xbot_interface as xb
from cartesian_interface.pyci_all import *
import numpy as np
import rospy

def add_wp(mat, time, wp_list):

    wp = pyci.WayPoint(mat, time)
    wp_list.append(wp)

def main():
    rospy.init_node('repair_box_pick')

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
    # exit(0)
    
    add_wp(Affine3(pos=[0.3116,  0.1199, -0.8384], rot=[0.6952, -0.3916,  0.5454,  0.2568]), 2*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.1835, -0.04574,  -0.8058], rot=[0.8107, -0.1026,  0.4771,  0.3236]), 2*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    #left hand rotation
    # translation: [  0.2466, 0.009412,  -0.7748]
    # rotation   : [-0.5472,  0.6069, -0.5726, 0.06626]


    add_wp(Affine3(pos=[0.2466, 0.009412,  -0.7748], rot=[-0.5472,  0.6069, -0.5726, 0.06626]), 1*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_2_tcp')
    
    waypoints_arm_2.clear()

    exit(0)


    
    #right hand rotation
    # translation: [ 0.3488,  0.1087, -0.7303]
    #rotation   : [ 0.7655, -0.2252,  0.4736,  0.3728]

    add_wp(Affine3(pos=[0.3488,  0.1087, -0.7303], rot=[0.7655, -0.2252,  0.4736,  0.3728]), 1*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)

    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    #right and left rotation
    # translation: [ 0.3357, 0.09681, -0.7826]
    # rotation   : [ 0.7122, 0.05171,  0.4219,  0.5587]
    # translation: [  0.2419, 0.001909,  -0.7709]
    # rotation   : [-0.4922,  0.6523, -0.5646,  0.1159]

    add_wp(Affine3(pos=[0.3357, 0.09681, -0.7826], rot=[0.7122, 0.05171,  0.4219,  0.5587]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.2419, 0.001909,  -0.7709], rot=[-0.4922,  0.6523, -0.5646,  0.1159]), 1*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_arm_1.clear()
    waypoints_arm_2.clear()


    #lift



    print('Motion completed!')
    exit(0)

    add_wp(Affine3(pos=[0.3116,  0.1199, -0.7384], rot=[0.6952, -0.3916,  0.5454,  0.2568]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.1835, -0.04574,  -0.7058], rot=[0.8107, -0.1026,  0.4771,  0.3236]), 1*time, waypoints_arm_2)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)

    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')



        
    

    # move object on the other side
    add_wp(Affine3(pos=[-0.3,    0, 1.97], rot=[-4.289e-09, -1.976e-08,     0.6066,      0.795]), 1*time, waypoints_torso)
    
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')

    # add_wp(Affine3(pos=[0.2686, -0.2368, -0.8264], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)

    # # move torso only
    # add_wp(Affine3(pos=[-0.3,    0, 1.97], rot=[ -0.01132, -0.005572,    0.4434,    0.8962]), 1*time, waypoints_torso)
    # ci.setWaypoints('torso_1', waypoints_torso)
    # ci.waitReachCompleted('torso_1')

    # waypoints_torso.clear()
    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # get close to object
    # add_wp(Affine3(pos=[0.2686,  0.2368, -0.8264], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.2686, -0.2368, -0.8264], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)

    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')

    # waypoints_torso.clear()
    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # # close object
    # add_wp(Affine3(pos=[0.2686,  0.16, -0.8264], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.2686, -0.16, -0.8264], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)

    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')

    # waypoints_torso.clear()
    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # # move object on the other side
    # add_wp(Affine3(pos=[0.5419,  0.1368, -0.5204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.5419, -0.1368, -0.5204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)
    # add_wp(Affine3(pos=[-0.3,    0, 1.97], rot=[0.0,  0.0,    -0.4505,     0.8928]), 1*time, waypoints_torso)
    
    # ci.setWaypoints('torso_1', waypoints_torso)
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')
    # ci.waitReachCompleted('torso_1')

    # waypoints_torso.clear()
    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # # open object
    # add_wp(Affine3(pos=[0.4419,  0.1368, -0.8204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.4419, -0.1368, -0.8204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)
    
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')

    # waypoints_torso.clear()
    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # # free object
    # add_wp(Affine3(pos=[0.4419,  0.3368, -0.8204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.4419, -0.3368, -0.8204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)
    
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')

    # waypoints_torso.clear()
    # waypoints_arm_1.clear()
    # waypoints_arm_2.clear()

    # # starting position
    # add_wp(Affine3(pos=[-0.3,    0, 1.97], rot=[0.0, 0.0, 0.0, 1.0]), 1*time, waypoints_torso)
    # add_wp(Affine3(pos=[0.5419,  0.4373, -0.5204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    # add_wp(Affine3(pos=[0.5419, -0.4373, -0.5204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)
    
    # ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    # ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    # ci.setWaypoints('torso_1', waypoints_torso)
    # ci.waitReachCompleted('arm_1_tcp')
    # ci.waitReachCompleted('arm_2_tcp')
    # ci.waitReachCompleted('torso_1')



    


    # niter = 1

    # t0 = rospy.Time.now()
    #
    # while not rospy.is_shutdown():
    #     print('Started loop ', niter, ', elapsed time ', (rospy.Time.now() - t0).to_sec())
    #     niter += 1
    #
    #     q1 = np.array([-2.5, -2.5, -2.5, 0.7, -2.7, -2.7, -2.7])
    #     move_to_q(robot, q0, q1, time)
    #
    #     q2 = np.array([0.0, -1.5, 0.0, -1.0, 0.0, -1.4, 0.0])
    #     move_to_q(robot, q1, q2, time)
    #
    #     q3 = np.array([2.5, -0.5, 2.5, -2.3, 2.7, 2.0, 2.7])
    #     move_to_q(robot, q2, q3, time)
    #
    #     move_to_q(robot, q3, q0, time)
    #
    #
    #
    # print('Exiting..')


if __name__ == '__main__':
    main()