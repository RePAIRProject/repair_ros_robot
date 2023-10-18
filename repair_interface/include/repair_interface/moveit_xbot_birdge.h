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
#include <memory>
#include <chrono>

class JointTrajectoryExecutor
{
  public:
    JointTrajectoryExecutor(ros::NodeHandle nh, std::string arm_controller_name, double goal_execution_timeout, double joint_angle_tolerance);

    virtual ~JointTrajectoryExecutor();

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
    double goal_execution_timeout_; // goal execution timeout in seconds
    std::string arm_name_; // name of arm
};

class MoveitXbotBridge
{
  public:
    double goal_execution_timeout_ = 5.0; // goal execution timeout in seconds
    double joint_angle_tolerance_ = 0.01; // joint angle tolerance in radians

    MoveitXbotBridge(ros::NodeHandle nh);

    virtual ~MoveitXbotBridge();

    ros::NodeHandle nh_;

    // repair controller names
    std::string arm_1_controller_name_ = "/arm_1_trajectory_controller/";
    std::string arm_2_controller_name_ = "/arm_2_trajectory_controller/";

    // trajectory executors
    std::shared_ptr<JointTrajectoryExecutor> arm_1_trajectory_executor_;
    std::shared_ptr<JointTrajectoryExecutor> arm_2_trajectory_executor_;
};


#endif // MOVEIT_XBOT_BRIDGE_H