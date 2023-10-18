#include "repair_interface/moveit_xbot_birdge.h"

JointTrajectoryExecutor::JointTrajectoryExecutor(ros::NodeHandle nh, std::string arm_controller_name, double goal_execution_timeout, double joint_angle_tolerance):
    nh_(nh),
    follow_joint_trajectory_as_(nh, arm_controller_name + "follow_joint_trajectory", boost::bind(&JointTrajectoryExecutor::executeCB, this, _1), false)
{
    // set goal execution timeout
    goal_execution_timeout_ = goal_execution_timeout;

    // set joint angle tolerance
    joint_angle_tolerance_ = joint_angle_tolerance;

    // subscribers
    joint_state_sub_ = nh_.subscribe("/xbotcore/joint_states", 1, &JointTrajectoryExecutor::jointStateCB, this);

    // publishers
    xbot_joint_command_pub_ = nh_.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1);

    // start action server
    follow_joint_trajectory_as_.start();

    ROS_INFO("Joint trajectory executor for %s started!", arm_controller_name.c_str());
}

JointTrajectoryExecutor::~JointTrajectoryExecutor()
{
}

void JointTrajectoryExecutor::jointStateCB(const xbot_msgs::JointState::ConstPtr& msg)
{
    current_joint_state_ = *msg;
}

void JointTrajectoryExecutor::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
    ROS_INFO("Received trajectory from MoveIt!");

    // get joint names from goal
    std::vector<std::string> joint_names = goal->trajectory.joint_names;

    // get trajectory points from goal
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = goal->trajectory.points;

    // length of trajectory
    ROS_INFO("Trajectory length: %d", (int)trajectory_points.size());

    // get current robot state
    std::vector<float> current_joint_positions = current_joint_state_.link_position;

    ROS_INFO("Executing trajectory...");

    // loop through trajectory points
    for (int i = 0; i < trajectory_points.size(); i++)
    {
        // get joint positions from trajectory point
        std::vector<double> joint_positions = trajectory_points[i].positions;

        // publish joint command
        publishJointCommand(joint_names, joint_positions);

        // wait for joint positions to be reached
        bool reached = false;

        // TODO: check if this is the best way to do this to avoid infinite loop
        // get the current time
        auto start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        while (!reached && (std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) - start_time) < goal_execution_timeout_)
        {
            // check if joint positions have been reached
            reached = true;
            for (int j = 0; j < joint_positions.size(); j++)
            {
                if (fabs(joint_positions[j] - current_joint_positions[j]) > joint_angle_tolerance_)
                {
                    reached = false;
                    break;
                }
            }

            // get current robot state
            current_joint_positions = current_joint_state_.link_position;

            // sleep
            ros::Duration(trajectory_points[i].time_from_start).sleep();
        }

    }

    ROS_INFO("Trajectory execution complete!");

    // mark the goal as succeeded
    follow_joint_trajectory_as_.setSucceeded();
}

void JointTrajectoryExecutor::publishJointCommand(std::vector<std::string> joint_names, std::vector<double> joint_positions)
{
    // create joint command message
    xbot_msgs::JointCommand joint_command;

    // set joint names
    joint_command.name = joint_names;

    // set joint positions
    joint_command.position = std::vector<float>(joint_positions.begin(), joint_positions.end());

    // publish joint command
    xbot_joint_command_pub_.publish(joint_command);
}

MoveitXbotBridge::MoveitXbotBridge(ros::NodeHandle nh):
    nh_(nh)
{
    arm_1_trajectory_executor_ = std::make_shared<JointTrajectoryExecutor>(nh_, arm_1_controller_name_, goal_execution_timeout_, joint_angle_tolerance_);
    arm_2_trajectory_executor_ = std::make_shared<JointTrajectoryExecutor>(nh_, arm_2_controller_name_, goal_execution_timeout_, joint_angle_tolerance_);
    torso_trajectory_executor_ = std::make_shared<JointTrajectoryExecutor>(nh_, torso_controller_name_, goal_execution_timeout_, joint_angle_tolerance_);
}

MoveitXbotBridge::~MoveitXbotBridge()
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_xbot_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Starting MoveIt! Xbot Bridge...");
    MoveitXbotBridge moveit_xbot_bridge(nh);

    ros::spin();

    return 0;
}