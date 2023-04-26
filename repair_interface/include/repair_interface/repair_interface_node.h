#ifndef REPAIR_INTERFACE_H
#define REPAIR_INTERFACE_H

// include moveitclient
#include "repair_interface/moveit_client.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>

#include <repair_interface/GetCurrentPose.h>
#include <repair_interface/MoveArmToPose.h>
#include <repair_interface/MoveBothArms.h>
#include <repair_interface/GripperCommand.h>

class RePairInterface
{
    public:
        RePairInterface(ros::NodeHandle nh);

        virtual ~RePairInterface();

        ros::NodeHandle nh_;

        // tf listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // change to false later
        bool debug_mode_ = true;

        // moveit client
        std::shared_ptr<MoveitClient> moveit_client_;

        // services
        ros::ServiceServer getCurrentPoseService_;
        ros::ServiceServer moveArmToPoseService_;
        ros::ServiceServer moveBothArmsToPoseService_;
        ros::ServiceServer gripperCommandService_;

        // service callbacks
        bool getCurrentPoseServiceCB(repair_interface::GetCurrentPose::Request &req, repair_interface::GetCurrentPose::Response &res);
        bool moveArmPoseServiceCB(repair_interface::MoveArmToPose::Request &req, repair_interface::MoveArmToPose::Response &res);
        bool moveBothArmsPoseServiceCB(repair_interface::MoveBothArms::Request &req, repair_interface::MoveBothArms::Response &res);
        bool gripperCommandServiceCB(repair_interface::GripperCommand::Request &req, repair_interface::GripperCommand::Response &res);

        void test();

};

#endif // REPAIR_INTERFACE_H