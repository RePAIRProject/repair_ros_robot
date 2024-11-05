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


   

    # homing_arm_r = [0.5, 0.5, -0.5, 1.0, 0.5, 0.5, 1.0] # arm_1 
    # homing_arm_l = [-0.5, -0.5, 0.5, -1.0, -0.5, -0.5, -1.0] # arm_2

    # s = np.array([-1, -1, -1, -1, -1, -1, -1])
    robot = get_robot()
    q0 = robot.getMotorPosition()
    q1 = - np.array([q0[0], 1.0, 2.0])

    current_time = 0.0
    dt = 0.01
    max_time = 3.0
    niter = 0

    t0 = rospy.Time.now()


    while not rospy.is_shutdown():

        niter += 1
        print('Started loop ', niter, ', elapsed time ', (rospy.Time.now() - t0).to_sec())
        
        move_to_q(robot, q0, q1, 1)
        move_to_q(robot, q1, q0, 2)
    # robot.get
    # amplitude = 0.3
    # while current_time <= time:
    #     q = q0 + [0, amplitude * np.sin(0.5 * current_time), amplitude * np.sin(0.5 * current_time)]
    #     print(q)
    #     robot.setPositionReference(q)
    #     robot.move()
    #     rospy.sleep(rospy.Duration(dt))
    #     current_time += dt

    

    print('Exiting..')


if __name__ == '__main__':
    main()
