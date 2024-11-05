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

    q1[la_idx:(la_idx + 7)] = la_q[:7]
    q1[ra_idx:(ra_idx + 7)] = ra_q[:7]

    return q1

    

def main():

    rospy.init_node('repair_stress_test')

    time = 4

    s = np.array([-1, -1, -1, -1, -1, -1, -1])

    robot = get_robot()

    # Rise arms and send them to the back
    q0 = robot.getMotorPosition()

    niter = 1
    t0 = rospy.Time.now()

    while not rospy.is_shutdown():

        print('Started loop ', niter, ', elapsed time ', (rospy.Time.now() - t0).to_sec())
        niter += 1

        # la_q_1 = -np.array([1.0, 0.16, 1.87, -0.61, 2.24, 1.58, 0.5])
        # la_q_1 = -np.array([0.8, 1.33, 1.61, -0.61, 2.24, 0.58, 0.5])
        la_q_1 = -np.array([0.8, 1.33, q0[2], -0.61, q0[4], 0.58, 0.5])
        
        
        q1 =la_to_q(robot, q0, la_q_1, s)

        q1[0] = 0.8
       


        print('Moving from q0 -> q1 ')
        move_to_q(robot, q0, q1, time)
        print('Moving from q0 -> q1 ')

        q0[0] = -0.8
        move_to_q(robot, q1, q0, time)

        print('Loop finished. Restarting...')
    

    print('Exiting..')


if __name__ == '__main__':
    main()
