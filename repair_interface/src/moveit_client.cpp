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
}

bool MoveitClient::moveToHome(enum ARM arm)
{
    bool success = false;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    switch (arm)
    {
        case ARM_1:
            move_group = move_group_arm_1;
            arm_plan = arm_1_plan;
            break;
        case ARM_2:
            move_group = move_group_arm_2;
            arm_plan = arm_2_plan;
            break;
        case BOTH_ARMS:
            move_group = move_group_both_arms;
            arm_plan = both_arms_plan;
            break;
        default:
            ROS_ERROR("Invalid arm selection");
            break;
    }

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