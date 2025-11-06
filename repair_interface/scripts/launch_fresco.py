#!/usr/bin/env python
import os
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

URDF_DIR = "/home/ws/src/repair_ros_robot/repair_urdf/urdf"
# on robot: '/home/repair/repair_ws/src/repair_ros_robot/repair_urdf/urdf/RPf_00205.urdf'

# name -> (x, y, z)
MODELS = {
    "RPf_00104": (-0.20, 0.30, 1.02),
    "RPf_00204": (0.00, 0.00, 1.02),
    "RPf_00205": (0.2, 0.3, 1.02),
}

def load_urdf_xml(model_name):
    urdf_path = os.path.join(URDF_DIR, f"{model_name}.urdf")
    if not os.path.isfile(urdf_path):
        rospy.logerr("URDF not found: %s", urdf_path)
        return None
    with open(urdf_path, "r") as f:
        return f.read()

def make_pose(x, y, z):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    # orientation left at zero (no rotation)
    return p

def spawn(model_name, pose, spawn_proxy):
    urdf_xml = load_urdf_xml(model_name)
    if urdf_xml is None:
        return
    try:
        resp = spawn_proxy(
            model_name=model_name,
            model_xml=urdf_xml,
            robot_namespace="",
            initial_pose=pose,
            reference_frame="world",
        )
        rospy.loginfo("[%s] Spawn status: %s", model_name, resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("[%s] Spawn failed: %s", model_name, e)

def main():
    rospy.init_node("spawn_frag_multi_node", anonymous=True)
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    spawn_proxy = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    for name, (x, y, z) in MODELS.items():
        pose = make_pose(x, y, z)
        spawn(name, pose, spawn_proxy)

if __name__ == "__main__":
    main()