#include "repair_interface/dawnik_bridge_node.h"

DawnikBridge::DawnikBridge(ros::NodeHandle nh) : nh_(nh) {
  dawnik_command_sub_arm1_ = nh_.subscribe(dawnik_command_topic_arm1_, 1,
                                      &DawnikBridge::dawnikCommandCBArm1, this);
  dawnik_command_sub_arm2_ = nh_.subscribe(dawnik_command_topic_arm2_, 1,
                                      &DawnikBridge::dawnikCommandCBArm2, this);
  dawnik_state_pub_arm1_ =
      nh_.advertise<control_msgs::JointTrajectoryControllerState>(
          dawnik_trajectory_state_topic_arm1_, 1);
  dawnik_state_pub_arm2_ =
      nh_.advertise<control_msgs::JointTrajectoryControllerState>(
          dawnik_trajectory_state_topic_arm2_, 1);

  xbot_command_pub_ =
      nh_.advertise<xbot_msgs::JointCommand>(xbot_command_topic_, 1);
  xbot_state_sub_ =
      nh_.subscribe(xbot_state_topic_, 1, &DawnikBridge::xbotStateCB, this);
}

DawnikBridge::~DawnikBridge() {}

void DawnikBridge::xbotStateCB(const xbot_msgs::JointState::ConstPtr &msg) {
  control_msgs::JointTrajectoryControllerState state_msg_arm1;
  control_msgs::JointTrajectoryControllerState state_msg_arm2;
  state_msg_arm1.header.stamp = ros::Time::now();
  state_msg_arm1.joint_names = joint_names_arm1_;
  state_msg_arm2.header.stamp = ros::Time::now();
  state_msg_arm2.joint_names = joint_names_arm2_;

  // actual
  for (int i = 0; i < joint_names_arm1_.size(); i++) {
    // get the index of the joint in the message
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_arm1_[i]);
    if (it != msg->name.end()) {
      int index = std::distance(msg->name.begin(), it);
      state_msg_arm1.actual.positions.push_back(msg->motor_position[index]);
      state_msg_arm1.actual.velocities.push_back(msg->motor_velocity[index]);

      state_msg_arm1.desired.positions.push_back(msg->position_reference[index]);
      state_msg_arm1.desired.velocities.push_back(msg->velocity_reference[index]);
    } else {
      ROS_ERROR("Joint %s not found in the message", joint_names_arm1_[i].c_str());
    }
    
  }

  for (int i = 0; i < joint_names_arm2_.size(); i++) {
    // get the index of the joint in the message
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_arm2_[i]);
    if (it != msg->name.end()) {
      int index = std::distance(msg->name.begin(), it);
      state_msg_arm2.actual.positions.push_back(msg->motor_position[index]);
      state_msg_arm2.actual.velocities.push_back(msg->motor_velocity[index]);

      state_msg_arm2.desired.positions.push_back(msg->position_reference[index]);
      state_msg_arm2.desired.velocities.push_back(msg->velocity_reference[index]);
    } else {
      ROS_ERROR("Joint %s not found in the message", joint_names_arm2_[i].c_str());
    }
  }

  dawnik_state_pub_arm1_.publish(state_msg_arm1);
  dawnik_state_pub_arm2_.publish(state_msg_arm2);
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