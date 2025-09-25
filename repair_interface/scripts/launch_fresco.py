#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import sys
import os

def spawn_urdf_model():
    rospy.init_node('spawn_frag2_node', anonymous=True)

    # Wait for the service
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    # Read URDF file
    urdf_path = os.path.join(
        os.getenv('HOME'), '/home/repair/repair_ws/src/repair_ros_robot/repair_urdf/urdf/RPf_00205.urdf'
    )
    with open(urdf_path, 'r') as file:
        urdf_xml = file.read()

    # Define pose
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 1.02

    try:
        resp = spawn_model(
            model_name='RPf_00205',
            model_xml=urdf_xml,
            robot_namespace='',
            initial_pose=pose,
            reference_frame='world'
        )
        rospy.loginfo("Spawn status: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn service call failed: %s", e)

if __name__ == '__main__':
    spawn_urdf_model()