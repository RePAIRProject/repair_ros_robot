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


    # translation: [-0.05334,  -0.6825,   0.1167]
    # rotation   : [  0.5859,   0.5951, -0.01868,   0.5497]
    torso_start, _, _ = ci.getPoseReference('torso_1')
    arm_1_start, _, _ = ci.getPoseReference('arm_1_tcp')
    arm_2_start, _, _ = ci.getPoseReference('arm_2_tcp')

    print(torso_start)
    print(arm_1_start)
    print(arm_2_start)

    # exit(0)

    # move torso
    add_wp(Affine3(pos=[-0.3, 0, 1.97], rot=[0.0, 0.0, 0.5125, 0.8587]), 1*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')
    waypoints_torso.clear()

    # left hand lowers
    add_wp(Affine3(pos=[0.3159,  0.4373, -0.7565], rot=[0.784, -0.2726, 0.554, -0.06394]), 1*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')
    waypoints_arm_1.clear()

    # push object
    add_wp(Affine3(pos=[-0.3, 0, 1.97], rot=[0.0, 0.0, -0.3489, 0.9372]), 1*time, waypoints_torso)
    add_wp(Affine3(pos=[0.22,  0.5027, -0.7565], rot=[ 0.7761, 0.04715,  0.5656, -0.2748]), 1*time, waypoints_arm_1)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('torso_1')
    ci.waitReachCompleted('arm_1_tcp')
    
    waypoints_torso.clear()
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()


    # raise left hand object
    add_wp(Affine3(pos=[0.3157,  0.5027, -0.4381], rot=[-0.4995,  0.2699, -0.4993,  0.6545]), 1*time, waypoints_arm_1)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.waitReachCompleted('arm_1_tcp')
    
    waypoints_torso.clear()
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    # move torso 
    add_wp(Affine3(pos=[-0.3,    0, 1.97], rot=[0.0,  0.0,    -0.6486,     0.7611]), 1*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')
    waypoints_torso.clear()
    
    # lower right hand

    add_wp(Affine3(pos=[0.4982, -0.2415, -0.7998], rot=[0.8374, -0.203, 0.4745, -0.1802]), 1*time, waypoints_arm_2)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_2_tcp')
    waypoints_arm_2.clear()

    # push second object
    add_wp(Affine3(pos=[-0.3,    0, 1.97], rot=[0.0,  0.0,    0.0,     1.0]), 1*time, waypoints_torso)
    add_wp(Affine3(pos=[ 0.4982, -0.1911, -0.7998], rot=[0.6581,  0.4639,  0.5926, 0.02169]), 1*time, waypoints_arm_2)
    
    
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('torso_1')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_torso.clear()
    waypoints_arm_2.clear()

    # starting position
    add_wp(Affine3(pos=[-0.3,    0, 1.97], rot=[0.0, 0.0, 0.0, 1.0]), 1*time, waypoints_torso)
    add_wp(Affine3(pos=[0.5419,  0.4373, -0.5204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.5419, -0.4373, -0.5204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)
    
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')
    ci.waitReachCompleted('torso_1')



    print('Motion completed!')


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