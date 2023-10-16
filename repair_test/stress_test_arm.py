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


def main():
    rospy.init_node('repair_stress_test')

    time = 4.0

    robot = get_robot()

    # Rise arms and send them to the back
    q0 = robot.getMotorPosition()

    niter = 1
    t0 = rospy.Time.now()

    while not rospy.is_shutdown():
        print('Started loop ', niter, ', elapsed time ', (rospy.Time.now() - t0).to_sec())
        niter += 1
    
        q1 = np.array([-2.5, 0.5, -2.5, -0.7, -2.7, -2.7, -2.7])
        # q1 = np.array([-2.5, -2.5, -2.5, 0.7, -2.7, -2.7, -2.7])
        move_to_q(robot, q0, q1, time)

        q2 = np.array([0.0, 1.5, 0.0, 1.0, 0.0, 0.0, 0.0])
        # q2 = np.array([0.0, -1.5, 0.0, -1.0, 0.0, -1.4, 0.0])
        move_to_q(robot, q1, q2, time)
        
        q3 = np.array([2.5, 2.5, 2.5, 2.3, 2.7, 2.0, 2.7])
        # q3 = np.array([2.5, -0.5, 2.5, -2.3, 2.7, 2.0, 2.7])
        move_to_q(robot, q2, q3, time)
    
        move_to_q(robot, q3, q0, time)
    


    print('Exiting..')


if __name__ == '__main__':
    main()