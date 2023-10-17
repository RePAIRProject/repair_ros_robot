#ifndef MOVEIT_XBOT_BRIDGE_H
#define MOVEIT_XBOT_BRIDGE_H

/**
 * Implements FollowJointTrajectoryAction interface.
 * Subscribes to /xbotcore/joint_state to get current robot state.
 * Receives trajectory from MoveIt! as a FollowJointTrajectoryActionGoal.
 * Loops through the trajectory and publishes each point to /xbotcore/command.
 * Publishes trajectory feedback to /move_group/follow_joint_trajectory/feedback.
 * Publishes trajectory result to /move_group/follow_joint_trajectory/result.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include <xbot_msgs/JointCommand.h>
#include <xbot_msgs/JointState.h>

#include <vector>
#include <string>

class MoveitXbotBridge
{
  public:
    MoveitXbotBridge(ros::NodeHandle nh);

    virtual ~MoveitXbotBridge();

    ros::NodeHandle nh_;

    // action server
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_as_;

    // subscribers
    ros::Subscriber joint_state_sub_;

    // publishers
    ros::Publisher xbot_joint_command_pub_;

    // callbacks
    void jointStateCB(const xbot_msgs::JointState::ConstPtr& msg);

    // action server callbacks
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

    // helper functions
    void publishJointCommand(std::vector<std::string> joint_names, std::vector<double> joint_positions);

  private:
    xbot_msgs::JointState current_joint_state_; // current robot state
    double joint_angle_tolerance_; // joint angle tolerance in radians
    // goal execution timeout in seconds
    double goal_execution_timeout_ = 5.0;
};


#endif // MOVEIT_XBOT_BRIDGE_H