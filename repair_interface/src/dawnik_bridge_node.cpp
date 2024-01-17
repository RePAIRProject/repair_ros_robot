#include "repair_interface/dawnik_bridge_node.h"

DawnikBridge::DawnikBridge(ros::NodeHandle nh) : nh_(nh) {
  dawnik_command_sub_arm1_ = nh_.subscribe(dawnik_command_topic_arm1_, 1,
                                      &DawnikBridge::dawnikCommandCBArm1, this);
  dawnik_command_sub_arm2_ = nh_.subscribe(dawnik_command_topic_arm2_, 1,
                                      &DawnikBridge::dawnikCommandCBArm2, this);
  dawnik_state_pub_ =
      nh_.advertise<control_msgs::JointTrajectoryControllerState>(
          dawnik_trajectory_state_topic_, 1);

  xbot_command_pub_ =
      nh_.advertise<xbot_msgs::JointCommand>(xbot_command_topic_, 1);
  xbot_state_sub_ =
      nh_.subscribe(xbot_state_topic_, 1, &DawnikBridge::xbotStateCB, this);
}

DawnikBridge::~DawnikBridge() {}

void DawnikBridge::xbotStateCB(const xbot_msgs::JointState::ConstPtr &msg) {
  control_msgs::JointTrajectoryControllerState state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.joint_names = msg->name;
  // actual
  state_msg.actual.positions = std::vector<double>(msg->motor_position.begin(),
                                                   msg->motor_position.end());
  state_msg.actual.velocities = std::vector<double>(msg->motor_velocity.begin(),
                                                    msg->motor_velocity.end());
  // desired
  state_msg.desired.positions = std::vector<double>(msg->position_reference.begin(),
                                                    msg->position_reference.end());
  state_msg.desired.velocities = std::vector<double>(msg->velocity_reference.begin(),
                                                     msg->velocity_reference.end());
  dawnik_state_pub_.publish(state_msg);
}

void DawnikBridge::dawnikCommandCBArm1(
    const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
  xbot_msgs::JointCommand joint_command;
  joint_command.header.stamp = ros::Time::now();
  joint_command.name = msg->joint_names;
  joint_command.position = std::vector<float>(msg->points[0].positions.begin(),
                                              msg->points[0].positions.end());
  joint_command.ctrl_mode =
      std::vector<uint8_t>(msg->points[0].positions.size(), 1);
  xbot_command_pub_.publish(joint_command);
}

void DawnikBridge::dawnikCommandCBArm2(
    const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
  xbot_msgs::JointCommand joint_command;
  joint_command.header.stamp = ros::Time::now();
  joint_command.name = msg->joint_names;
  joint_command.position = std::vector<float>(msg->points[0].positions.begin(),
                                              msg->points[0].positions.end());
  joint_command.ctrl_mode =
      std::vector<uint8_t>(msg->points[0].positions.size(), 1);
  xbot_command_pub_.publish(joint_command);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dawnik_bridge");
  ros::NodeHandle nh;
  DawnikBridge dawnik_bridge(nh);
  ros::spin();
  return 0;
}