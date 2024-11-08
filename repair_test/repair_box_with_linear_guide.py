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

    print(waypoints_torso)
    print(torso_start)
    print(arm_1_start)
    print(arm_2_start)

    add_wp(Affine3(pos=[-0.3, 0.4506,   1.97], rot=[6.931e-14, 2.359e-14,    0.4434,    0.8963]), 1*time, waypoints_torso)
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.waitReachCompleted('torso_1')

    waypoints_torso.clear()
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()
    


    # get close to object
    add_wp(Affine3(pos=[0.2686,  0.2368, -0.8264], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.2686, -0.2368, -0.8264], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)

    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_torso.clear()
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    # close object
    add_wp(Affine3(pos=[0.2686,  0.16, -0.8264], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.2686, -0.16, -0.8264], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)

    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_torso.clear()
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    # move object on the other side
    add_wp(Affine3(pos=[0.5419,  0.1368, -0.5204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 3*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.5419, -0.1368, -0.5204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 3*time, waypoints_arm_2)
    add_wp(Affine3(pos=[-0.3,    -0.4, 1.97], rot=[0.0,  0.0,    -0.4505,     0.8928]), 3*time, waypoints_torso)
    
    ci.setWaypoints('torso_1', waypoints_torso)
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')
    ci.waitReachCompleted('torso_1')

    waypoints_torso.clear()
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    # open object
    add_wp(Affine3(pos=[0.4419,  0.1368, -0.8204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.4419, -0.1368, -0.8204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)
    
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_torso.clear()
    waypoints_arm_1.clear()
    waypoints_arm_2.clear()

    # free object
    add_wp(Affine3(pos=[0.4419,  0.3368, -0.8204], rot=[0.5223,  -0.2147,   0.8194, -0.09796]), 1*time, waypoints_arm_1)
    add_wp(Affine3(pos=[0.4419, -0.3368, -0.8204], rot=[0.5223,  0.2147,  0.8194, 0.09796]), 1*time, waypoints_arm_2)
    
    ci.setWaypoints('arm_1_tcp', waypoints_arm_1)
    ci.setWaypoints('arm_2_tcp', waypoints_arm_2)
    ci.waitReachCompleted('arm_1_tcp')
    ci.waitReachCompleted('arm_2_tcp')

    waypoints_torso.clear()
    waypoints_arm_1.clear()
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