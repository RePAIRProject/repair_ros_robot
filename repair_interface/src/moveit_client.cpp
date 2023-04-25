#include <repair_interface/moveit_client.h>

MoveitClient::MoveitClient(ros::NodeHandle nh):
    nh_(nh)
{
    initMoveitClient();
}

MoveitClient::~MoveitClient()
{
}

void MoveitClient::initMoveitClient()
{
    move_group_arm_1 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_ARM_1);
    move_group_arm_2 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_ARM_2);

    move_group_hand_1 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_HAND_1);
    move_group_hand_2 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_HAND_2);

    move_group_both_arms = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_BOTH_ARMS);

    // get end effector link names for each arm
    std::string arm_1_end_effector_link = move_group_arm_1->getEndEffectorLink();
    std::string arm_2_end_effector_link = move_group_arm_2->getEndEffectorLink();

    // get end effector link names for both_arms group
    auto both_arms_arm_1_end_effector_link = move_group_both_arms->getEndEffectorLink();
}

bool MoveitClient::sendPoseToSingleArm(geometry_msgs::PoseStamped pose, enum ARM arm)
{
    bool success = false;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    
    getArmMoveGroupAndPlan(arm, move_group, arm_plan);

    move_group->setPoseTarget(pose);
    ROS_INFO("Planning to move arm to target pose");
    success = (move_group->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Moving arm to target pose");
        move_group->execute(arm_plan);
        // move_group->move(); TODO: test this
    }
    else
    {
        ROS_ERROR("Failed to move arm to target pose");
    }

    return success;
}

bool MoveitClient::sendPoseToBothArms(geometry_msgs::PoseStamped arm_1_pose, geometry_msgs::PoseStamped arm_2_pose)
{
    bool success = false;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::planning_interface::MoveGroupInterface::Plan both_arms_plan;
    
    move_group = move_group_both_arms;

    move_group->setPoseTarget(arm_1_pose, "arm_1_link_7");
    move_group->setPoseTarget(arm_2_pose, "arm_2_link_7");
    ROS_INFO("Planning to move both arms to target pose");
    success = (move_group->plan(both_arms_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Moving both arms to target pose");
        move_group->execute(both_arms_plan);
        // move_group->move(); TODO: test this
    }
    else
    {
        ROS_ERROR("Failed to move both arms to target pose");
    }

    return success;
}

bool MoveitClient::moveToHome(enum ARM arm)
{
    bool success = false;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    
    getArmMoveGroupAndPlan(arm, move_group, arm_plan);

    move_group->setJointValueTarget(move_group->getNamedTargetValues("home"));
    ROS_INFO("Planning to move arm to home position");
    success = (move_group->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Moving arm to home position");
        move_group->execute(arm_plan);
        // move_group->move(); TODO: test this
    }
    else
    {
        ROS_ERROR("Failed to move arm to home position");
    }

    return success;
}

bool MoveitClient::controlHand(enum HAND hand, enum HAND_STATE state)
{
    bool success = false;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_hand;
    moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
    switch (hand)
    {
        case HAND_1:
            move_group_hand = move_group_hand_1;
            hand_plan = hand_1_plan;
            break;
        case HAND_2:
            move_group_hand = move_group_hand_2;
            hand_plan = hand_2_plan;
            break;
        default:
            ROS_ERROR("Invalid hand selection");
            break;
    }

    std::string hand_state;
    switch (state)
    {
        case OPEN:
            hand_state = "open";
            break;
        case CLOSE:
            hand_state = "close";
            break;
        default:
            ROS_ERROR("Invalid hand state selection");
            break;
    }

    move_group_hand->setJointValueTarget(move_group_hand->getNamedTargetValues(hand_state));
    ROS_INFO("Planning to move hand to %s position", hand_state.c_str());

    success = (move_group_hand->plan(hand_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Moving hand to %s position", hand_state.c_str());
        move_group_hand->execute(hand_plan);
        // move_group_hand->move();
    }
    else
    {
        ROS_ERROR("Failed to move hand to %s position", hand_state.c_str());
    }

    return success;
}

std::vector<geometry_msgs::PoseStamped> MoveitClient::getCurrentPose()
{
    geometry_msgs::PoseStamped current_pose_arm_1 = move_group_arm_1->getCurrentPose();
    geometry_msgs::PoseStamped current_pose_arm_2 = move_group_arm_2->getCurrentPose();

    // create an array of doubles to store roll, pitch, yaw
    std::array<double, 3> rpy_arm_1;
    std::array<double, 3> rpy_arm_2;

    // convert quaternions to roll, pitch, yaw
    convertQuatToRPY(current_pose_arm_1, rpy_arm_1[0], rpy_arm_1[1], rpy_arm_1[2]);
    convertQuatToRPY(current_pose_arm_2, rpy_arm_2[0], rpy_arm_2[1], rpy_arm_2[2]);

    ROS_INFO("Current pose of arm 1: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f", 
                current_pose_arm_1.pose.position.x, 
                current_pose_arm_1.pose.position.y, 
                current_pose_arm_1.pose.position.z, 
                rpy_arm_1[0], rpy_arm_1[1], rpy_arm_1[2]);

    ROS_INFO("Current pose of arm 2: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
                current_pose_arm_2.pose.position.x, 
                current_pose_arm_2.pose.position.y, 
                current_pose_arm_2.pose.position.z, 
                rpy_arm_2[0], rpy_arm_2[1], rpy_arm_2[2]);

    // return both arm poses in a vector
    std::vector<geometry_msgs::PoseStamped> current_poses;
    current_poses.push_back(current_pose_arm_1);
    current_poses.push_back(current_pose_arm_2);

    return current_poses;
}

void MoveitClient::getArmMoveGroupAndPlan(enum ARM arm, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group, 
                                    moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    switch (arm)
    {
        case ARM::ARM_1:
            move_group = move_group_arm_1;
            plan = arm_1_plan;
            break;
        case ARM::ARM_2:
            move_group = move_group_arm_2;
            plan = arm_2_plan;
            break;
        default:
            ROS_ERROR("Invalid arm selection");
            break;
    }
}

void MoveitClient::convertQuatToRPY(geometry_msgs::PoseStamped pose, double& roll, double& pitch, double& yaw)
{
    tf2::Quaternion q(pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // convert to degrees
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
}