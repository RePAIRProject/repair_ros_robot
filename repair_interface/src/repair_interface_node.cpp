#include "repair_interface/repair_interface_node.h"

RePairInterface::RePairInterface(ros::NodeHandle nh):
    nh_(nh)
{
    // init moveit client
    moveit_client_ = std::make_shared<MoveitClient>(nh_);

    // init tf listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // init services
    getCurrentPoseService_ = nh_.advertiseService("/get_current_pose_srv", &RePairInterface::getCurrentPoseServiceCB, this);
    moveArmToPoseService_ = nh_.advertiseService("/move_arm_to_pose_srv", &RePairInterface::moveArmPoseServiceCB, this);
    moveBothArmsToPoseService_ = nh_.advertiseService("/move_both_arms_srv", &RePairInterface::moveBothArmsPoseServiceCB, this);
    gripperCommandService_ = nh_.advertiseService("/gripper_command_srv", &RePairInterface::gripperCommandServiceCB, this);

    ROS_INFO("RePair interface node initialized");
}

RePairInterface::~RePairInterface()
{
}

bool RePairInterface::gripperCommandServiceCB(repair_interface::GripperCommand::Request &req, repair_interface::GripperCommand::Response &res)
{
    ROS_INFO("Recieved request to command gripper");

    // read gripper command from request
    int32_t command = req.command;
    int32_t hand = req.hand;

    MoveitClient::HAND hand_enum = MoveitClient::HAND(hand);
    MoveitClient::HAND_STATE gripper_command = MoveitClient::HAND_STATE(command);

    // send gripper command to gripper
    bool success = moveit_client_->controlHand(hand_enum, gripper_command);

    if (success)
    {
        ROS_INFO("Successfully commanded gripper");
    }
    else
    {
        ROS_ERROR("Failed to command gripper");
    }

    res.success = success;

    return true;
}

bool RePairInterface::getCurrentPoseServiceCB(repair_interface::GetCurrentPose::Request &req, repair_interface::GetCurrentPose::Response &res)
{
    ROS_INFO("Recieved request to get current pose");

    // get current pose
    std::vector<geometry_msgs::PoseStamped> current_pose = moveit_client_->getCurrentPose();

    // return current pose
    res.current_pose_1 = current_pose[0];
    res.current_pose_2 = current_pose[1];

    return true;
}

bool RePairInterface::moveArmPoseServiceCB(repair_interface::MoveArmToPose::Request &req, repair_interface::MoveArmToPose::Response &res)
{
    geometry_msgs::PoseStamped target_pose = req.target_pose;
    int32_t arm = req.arm;
    
    ROS_INFO("Recieved request to move arm_%d to target pose", arm);

    MoveitClient::ARM arm_enum = MoveitClient::ARM(arm);

    // send target pose to arm
    bool success = moveit_client_->sendPoseToSingleArm(target_pose, arm_enum);

    if (success)
    {
        ROS_INFO("Successfully moved arm_%d to target pose", arm);
    }
    else
    {
        ROS_ERROR("Failed to move arm_%d to target pose", arm);
    }

    res.success = success;
    
    std::vector<geometry_msgs::PoseStamped> current_pose = moveit_client_->getCurrentPose();
    res.current_pose = current_pose[arm];

    return true;
}

bool RePairInterface::moveBothArmsPoseServiceCB(repair_interface::MoveBothArms::Request &req, repair_interface::MoveBothArms::Response &res)
{
    ROS_INFO("Recieved request to move both arms to target pose");

    // read target pose from request
    geometry_msgs::PoseStamped target_pose_1 = req.target_pose_1;
    geometry_msgs::PoseStamped target_pose_2 = req.target_pose_2;

    // send target pose to both arms
    bool success = moveit_client_->sendPoseToBothArms(target_pose_1, target_pose_2);

    if (success)
    {
        ROS_INFO("Successfully moved both arms to target pose");
    }
    else
    {
        ROS_ERROR("Failed to move both arms to target pose");
    }

    res.success = success;
    
    std::vector<geometry_msgs::PoseStamped> current_pose = moveit_client_->getCurrentPose();

    res.current_pose_1 = current_pose[0];
    res.current_pose_2 = current_pose[1];

    return true;
}

void RePairInterface::test()
{
    ROS_INFO("Initializing test... Alles gut!");

    // test moveit client for both arms

    // get current pose
    std::vector<geometry_msgs::PoseStamped> current_pose = moveit_client_->getCurrentPose();

    // go up 5 cm
    geometry_msgs::PoseStamped target_pose_1 = current_pose[0];
    target_pose_1.pose.position.z += 0.05;


    // send target pose to both arms
    bool success = moveit_client_->sendPoseToSingleArm(target_pose_1, MoveitClient::ARM::ARM_1);

    if (success)
    {
        ROS_INFO("Successfully moved arm_1 to target pose");
    }
    else
    {
        ROS_ERROR("Failed to move arm_1 to target pose");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "repair_interface");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    RePairInterface repair_interface(nh);

    // repair_interface.test();

    ros::waitForShutdown();
    
    return 0;
}