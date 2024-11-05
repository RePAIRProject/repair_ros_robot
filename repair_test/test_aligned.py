#!/usr/bin/env python

import xbot_interface.config_options as co
import xbot_interface.xbot_interface as xb
import numpy as np
import rospy


def get_robot():
    cfg = co.ConfigOptions()
    prefix = 'xbotcore/'
    urdf = rospy.get_param(prefix + 'robot_description')
    srdf = rospy.get_param(prefix + 'robot_description_semantic')

    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_bool_parameter('is_model_floating_base', False)
    robot = xb.RobotInterface(cfg)
    robot.sense()
    robot.setControlMode(xb.ControlMode.Position())

    return robot

def move_to_q(robot, q0, q1, time):

    current_time = 0.0
    dt = 0.01

    while current_time <= time:
        alpha = current_time/time
        alpha = alpha**2*(2-alpha)**2
        q = alpha*q1 + (1-alpha)*q0
        robot.setPositionReference(q)
        robot.move()
        rospy.sleep(rospy.Duration(dt))
        current_time += dt

# def la_to_q(robot, q, la_q, signs):

    # la_idx = robot.getDofIndex('j_arm_1_1')
    # ra_idx = robot.getDofIndex('j_arm_2_1')

    # ra_q = la_q * signs

    # q1 = np.array(q)

    # q1[la_idx:(la_idx + 6)] = la_q[:6]
    # q1[ra_idx:(ra_idx + 6)] = ra_q[:6]

    # return q1

    

def main():

    rospy.init_node('repair_stress_test')

    joint_index = 0

    robot = get_robot()
    q0 = robot.getMotorPosition()

    q_min, q_max = robot.getJointLimits()

    current_time = 0.0
    amplitude = 1.5
    dt = 0.1
    max_time = 20
    frequency = 0.05
    # offset = (max_val + min_val) / 2
    q = q0

    while current_time <= max_time:

        q[0] = amplitude * np.sin( 2 * np.pi * frequency * current_time)

        print(q[0])

        robot.setPositionReference(q)
        robot.move()
        rospy.sleep(rospy.Duration(dt))
        current_time += dt

    exit()


    print('Exiting..')


if __name__ == '__main__':
    main()
