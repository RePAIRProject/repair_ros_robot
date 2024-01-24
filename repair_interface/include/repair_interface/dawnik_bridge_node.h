#ifndef DAWNIK_BRIDGE_H
#define DAWNIK_BRIDGE_H

#include <ros/ros.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "xbot_msgs/JointState.h"
#include "xbot_msgs/JointCommand.h"

class DawnikBridge
{
  public:
    DawnikBridge(ros::NodeHandle nh);
    virtual ~DawnikBridge();

    ros::NodeHandle nh_;

    // topic names
    std::string dawnik_command_topic_arm1_ = "/dawnik/arm1/command";
    std::string dawnik_command_topic_arm2_ = "/dawnik/arm2/command";
    std::string dawnik_trajectory_state_topic_ = "/dawnik_bridge/trajectory_state";

    std::string xbot_command_topic_ = "/xbotcore/command";
    std::string xbot_state_topic_ = "/xbotcore/joint_states";

    // dawnik
    ros::Subscriber dawnik_command_sub_arm1_;
    ros::Subscriber dawnik_command_sub_arm2_;
    ros::Publisher dawnik_state_pub_;

    // xbot
    ros::Publisher xbot_command_pub_;
    ros::Subscriber xbot_state_sub_;

    // callbacks
    void dawnikCommandCBArm1(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void dawnikCommandCBArm2(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void xbotStateCB(const xbot_msgs::JointState::ConstPtr& msg);
};

#endif // DAWNIK_BRIDGE_H