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
    rospy.init_node('repair_stress_test')

    time = 3.0

    # robot = get_robot()
    # Rise arms and send them to the back
    # q0 = robot.getMotorPosition()

    ci = pyci.CartesianInterfaceRos()
    ci.update()

    waypoints = []


    # translation: [-0.05334,  -0.6825,   0.1167]
    # rotation   : [  0.5859,   0.5951, -0.01868,   0.5497]
    arm_start, _, _ = ci.getPoseReference('arm_1_tcp')
    add_wp(Affine3(pos=[-0.5082, -0.3903, 0.06521], rot=[ 0.5301,  0.3273, -0.2502,  0.7412]), 1*time, waypoints)
    add_wp(Affine3(pos=[-0.2159, -0.3903, 0.7449], rot=[ 0.5301,  0.3273, -0.2502,  0.7412]), 2*time, waypoints)
    add_wp(Affine3(pos=[0.4454, -0.3903, 0.1508], rot=[ 0.5301,  0.3273, -0.2502,  0.7412]), 3*time, waypoints)
    add_wp(Affine3(pos=[-0.08358, -0.648, 0.01989], rot=[ 0.588,  0.3463, -0.2231,  0.6961]), 4*time, waypoints)
    add_wp(arm_start, 5*time, waypoints)

    ci.setWaypoints('arm_1_tcp', waypoints)
    ci.waitReachCompleted('arm_1_tcp')

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