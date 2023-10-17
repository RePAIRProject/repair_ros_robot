#include "repair_interface/moveit_xbot_birdge.h"

MoveitXbotBridge::MoveitXbotBridge(ros::NodeHandle nh):
    nh_(nh),
    follow_joint_trajectory_as_(nh_, "follow_joint_trajectory", boost::bind(&MoveitXbotBridge::executeCB, this, _1), false)
{
    // subscribers
    joint_state_sub_ = nh_.subscribe("/xbotcore/joint_state", 1, &MoveitXbotBridge::jointStateCB, this);

    // publishers
    xbot_joint_command_pub_ = nh_.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1);

    // start action server
    follow_joint_trajectory_as_.start();
}

MoveitXbotBridge::~MoveitXbotBridge()
{
}

void MoveitXbotBridge::jointStateCB(const xbot_msgs::JointState::ConstPtr& msg)
{
    current_joint_state_ = *msg;
}

void MoveitXbotBridge::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
    ROS_INFO("Received trajectory from MoveIt!");

    // accept the new goal
    follow_joint_trajectory_as_.acceptNewGoal();

    // get joint names from goal
    std::vector<std::string> joint_names = goal->trajectory.joint_names;

    // get trajectory points from goal
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = goal->trajectory.points;

    // get joint angle tolerance from goal
    joint_angle_tolerance_ = goal->path_tolerance[0].position;

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
        time_t start_time = time(NULL);
        while (!reached && (time(NULL) - start_time) < goal_execution_timeout_)
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

void MoveitXbotBridge::publishJointCommand(std::vector<std::string> joint_names, std::vector<double> joint_positions)
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_xbot_bridge");
    ros::NodeHandle nh;

    MoveitXbotBridge moveit_xbot_bridge(nh);

    ros::spin();

    return 0;
}