#ifndef DAWNIK_BRIDGE_H
#define DAWNIK_BRIDGE_H

#include <control_msgs/JointTrajectoryControllerState.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "xbot_msgs/JointCommand.h"
#include "xbot_msgs/JointState.h"

class DawnikBridge {
 public:
  DawnikBridge(ros::NodeHandle nh);
  virtual ~DawnikBridge();

  ros::NodeHandle nh_;

  // topic names
  std::string dawnik_command_topic_arm1_ = "/dawnik/arm1/command";
  std::string dawnik_command_topic_arm2_ = "/dawnik/arm2/command";
  std::string dawnik_trajectory_state_topic_arm1_ =
      "/dawnik_bridge/arm1/trajectory_state";
  std::string dawnik_trajectory_state_topic_arm2_ =
      "/dawnik_bridge/arm2/trajectory_state";

  std::string xbot_command_topic_ = "/xbotcore/command";
  std::string xbot_state_topic_ = "/xbotcore/joint_states";

//   std::vector<std::string> joint_names_arm1_ = {
//       "j_torso_1", "j_arm_1_1", "j_arm_1_2", "j_arm_1_3",
//       "j_arm_1_4", "j_arm_1_5", "j_arm_1_6", "j_arm_1_7"};
//   std::vector<std::string> joint_names_arm2_ = {
//       "j_torso_1", "j_arm_2_1", "j_arm_2_2", "j_arm_2_3",
//       "j_arm_2_4", "j_arm_2_5", "j_arm_2_6", "j_arm_2_7"};

  std::vector<std::string> joint_names_arm1_ = {
      "j_arm_1_1", "j_arm_1_2", "j_arm_1_3",
      "j_arm_1_4", "j_arm_1_5", "j_arm_1_6", "j_arm_1_7"};
  std::vector<std::string> joint_names_arm2_ = {
      "j_arm_2_1", "j_arm_2_2", "j_arm_2_3",
      "j_arm_2_4", "j_arm_2_5", "j_arm_2_6", "j_arm_2_7"};

  // dawnik
  ros::Subscriber dawnik_command_sub_arm1_;
  ros::Subscriber dawnik_command_sub_arm2_;
  ros::Publisher dawnik_state_pub_arm1_;
  ros::Publisher dawnik_state_pub_arm2_;

  // xbot
  ros::Publisher xbot_command_pub_;
  ros::Subscriber xbot_state_sub_;

  // callbacks
  void dawnikCommandCBArm1(
      const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  void dawnikCommandCBArm2(
      const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  void xbotStateCB(const xbot_msgs::JointState::ConstPtr& msg);
};

#endif  // DAWNIK_BRIDGE_H