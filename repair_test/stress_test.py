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

def la_to_q(robot, q, la_q, signs):

    la_idx = robot.getDofIndex('j_arm_1_1')
    ra_idx = robot.getDofIndex('j_arm_2_1')

    ra_q = la_q * signs

    q1 = np.array(q)

    q1[la_idx:(la_idx + 6)] = la_q[:6]
    q1[ra_idx:(ra_idx + 6)] = ra_q[:6]

    return q1

    

def main():

    rospy.init_node('repair_stress_test')

    time = 1.5

    # homing_arm_r = [0.5, 0.5, -0.5, 1.0, 0.5, 0.5, 1.0] # arm_1 
    # homing_arm_l = [-0.5, -0.5, 0.5, -1.0, -0.5, -0.5, -1.0] # arm_2

    s = np.array([-1, -1, -1, -1, -1, -1, -1])

    robot = get_robot()

    # Rise arms and send them to the back
    q0 = robot.getMotorPosition()

    niter = 1
    t0 = rospy.Time.now()

    while not rospy.is_shutdown():

        print('Started loop ', niter, ', elapsed time ', (rospy.Time.now() - t0).to_sec())
        niter += 1

        la_q_1 = -np.array([1.0, 0.0, -0.5, 1.0, 2.0, 1.9, 0.0])
        la_q_2 = -np.array([1.0, 1.0, -2.1, 0.5, -0.5, -1.5, -2.7])
        la_q_3 = -np.array([0.5, 0.8, -0.7, 2.0, 0.5, 1.5, 1])
        la_q_4 = -np.array([0.5, 0.4, -1.0, 0.7, 1.8, -1.0, -2.5])
        
        q1 =la_to_q(robot, q0, la_q_1, s)
        q2 =la_to_q(robot, q0, la_q_2, s)
        q3 =la_to_q(robot, q0, la_q_3, s)
        q4 =la_to_q(robot, q0, la_q_4, s)

        print('Moving from q0 -> q1 ')
        move_to_q(robot, q0, q1, time)
        print('Moving from q1 -> q2 ')
        move_to_q(robot, q1, q2, 1.5*time)
        print('Moving from q2 -> q3 ')
        move_to_q(robot, q2, q3, 2*time)
        print('Moving from q3 -> q4 ')
        move_to_q(robot, q3, q4, 2.5*time)
        print('Moving from q4 -> q5 ')
        move_to_q(robot, q4, q0, 2*time)
        print('Loop finished. Restarting...')
    

    print('Exiting..')


if __name__ == '__main__':
    main()