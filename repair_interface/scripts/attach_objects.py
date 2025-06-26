#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def attach_links(model_1="repair", link_1="arm_2_7", model_2="RPf_00205", link_2="RPf_00204_link"):
    # Wait for the attach service to be available
    rospy.wait_for_service('/link_attacher_node/attach')
    try:
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

        req = AttachRequest()
        req.model_name_1 = model_1
        req.link_name_1 = link_1
        req.model_name_2 = model_2
        req.link_name_2 = link_2

        result = attach_srv(req)
        if result.ok:
            rospy.loginfo("Links successfully attached!")
            return True
        else:
            rospy.logwarn("Attach failed!")
            return False

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def detach_links(model_1, link_1, model_2, link_2):
    """
    Detach link_2 of model_2 from link_1 of model_1 using gazebo_ros_link_attacher.

    Parameters:
        model_1 (str): Name of the first model (e.g., 'repair')
        link_1  (str): Name of the first model's link (e.g., 'arm_2_7')
        model_2 (str): Name of the second model (e.g., 'RPf_00205')
        link_2  (str): Name of the second model's link (e.g., 'RPf_00204_link')

    Returns:
        bool: True if detached successfully, False otherwise
    """
    rospy.wait_for_service('/link_attacher_node/detach')
    try:
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        req = AttachRequest()
        req.model_name_1 = model_1
        req.link_name_1 = link_1
        req.model_name_2 = model_2
        req.link_name_2 = link_2

        resp = detach_srv(req)
        return resp.ok
    except rospy.ServiceException as e:
        rospy.logerr("Detach service call failed: %s" % e)
        return False

if __name__ == '__main__':
    attach_links("repair", "arm_2_7", "RPf_00205", "RPf_00204_link")
    
    # success = detach_links("repair", "arm_2_7", "RPf_00205", "RPf_00204_link")
    # if success:
    #     rospy.loginfo("Successfully detached.")
    # else:
    #     rospy.logwarn("Failed to detach.")