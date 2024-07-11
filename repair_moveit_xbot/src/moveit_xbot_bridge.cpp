#include "repair_moveit_xbot/moveit_xbot_bridge.h"

JointTrajectoryExecutor::JointTrajectoryExecutor(ros::NodeHandle nh, std::string arm_controller_name, double goal_execution_timeout, double joint_angle_tolerance, std::shared_ptr<xbot_msgs::JointState> current_joint_state_ptr):
    nh_(nh),
    follow_joint_trajectory_as_(nh, arm_controller_name + "follow_joint_trajectory", boost::bind(&JointTrajectoryExecutor::executeCB, this, _1), false)
{
    // set goal execution timeout
    goal_execution_timeout_ = goal_execution_timeout;

    // set joint angle tolerance
    joint_angle_tolerance_ = joint_angle_tolerance;

    // subscribers
    // joint_state_sub_ = nh_.subscribe("/xbotcore/joint_states", 1, &JointTrajectoryExecutor::jointStateCB, this);

    // publishers
    xbot_joint_command_pub_ = nh_.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1);

    // start action server
    follow_joint_trajectory_as_.start();

    current_joint_state_ptr_ = current_joint_state_ptr;

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
    int amount_of_trajectory_points = (int)trajectory_points.size();
    ROS_INFO("Trajectory length: %d", amount_of_trajectory_points);

    // get current robot state
    std::vector<float> current_joint_positions = current_joint_state_ptr_->link_position;

    ROS_INFO("Executing trajectory...");

    std::vector<double> joint_positions;

    // ros::Duration previous_time_from_start(0.0); // Initialize with 0.0 seconds

    // loop through trajectory points
    for (int i = 0; i < trajectory_points.size(); i++)
    {
        // get joint positions from trajectory point
        joint_positions = trajectory_points[i].positions;

        // ROS_INFO("Trajectory time from start: %f", trajectory_points[i].time_from_start.toSec());

        // // Calculate time difference if it's not the first point
        // if (i > 0)
        // {
        //     ros::Duration time_diff = trajectory_points[i].time_from_start - previous_time_from_start;
        //     ROS_INFO("Time difference: %f", time_diff.toSec());
        //     // Sleep for the time difference
        //     time_diff.sleep();
        // }

        // publish joint command
        publishJointCommand(joint_names, joint_positions);

        ros::Duration(0.1).sleep();

        // Update previous_time_from_start
        // previous_time_from_start = trajectory_points[i].time_from_start;

        // TODO: check if this is the best way to do this to avoid infinite loop
        // get the current time
        /*
        auto start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        while (!reached) //&& (std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) - start_time) < goal_execution_timeout_)
        {
            current_joint_positions = current_joint_state_ptr_->link_position;
            // check if joint positions have been reached
            reached = true;

            for (int j = 0; j < joint_positions.size(); j++)
            {
                ROS_INFO("joint_no/joint_amount = %d/%d", j, joint_positions.size()-1);
                ROS_INFO("current_joint_positions[j] = %f", current_joint_positions[j]);
                ROS_INFO("joint_positions[j] = %f", joint_positions[j]);
                ROS_INFO("joint_angle_tolerance_ = %f", joint_angle_tolerance_);
                double test = fabs(joint_positions[j] - current_joint_positions[j]);
                ROS_INFO("calc = %f", test);
            }

            for (int j = 0; j < joint_positions.size(); j++)
            {
                // ROS_INFO("joint_no/joint_amount = %d/%d", j, joint_positions.size()-1);
                // ROS_INFO("current_joint_positions[j] = %f", current_joint_positions[j]);
                // ROS_INFO("joint_positions[j] = %f", joint_positions[j]);
                // ROS_INFO("joint_angle_tolerance_ = %f", joint_angle_tolerance_);
                // double test = fabs(joint_positions[j] - current_joint_positions[j]);
                // ROS_INFO("calc = %f", test);
                if (fabs(joint_positions[j] - current_joint_positions[j]) > joint_angle_tolerance_)
                {
                    reached = false;
                    break;
                }
            }

            // sleep
            // ros::Duration(0.001).sleep();
        }
        */

    }
/*
    // Check if final joint positions were reached
    bool reached = true;
    current_joint_positions = current_joint_state_ptr_->link_position;
    joint_positions = trajectory_points[amount_of_trajectory_points].positions;

    for (int j = 0; j < joint_positions.size(); j++)
    {
        // ROS_INFO("joint_no/joint_amount = %d/%d", j, joint_positions.size()-1);
        // ROS_INFO("current_joint_positions[j] = %f", current_joint_positions[j]);
        // ROS_INFO("joint_positions[j] = %f", joint_positions[j]);
        // ROS_INFO("joint_angle_tolerance_ = %f", joint_angle_tolerance_);
        // double test = fabs(joint_positions[j] - current_joint_positions[j]);
        // ROS_INFO("calc = %f", test);
        if (fabs(joint_positions[j] - current_joint_positions[j]) > joint_angle_tolerance_)
        {
            reached = false;
            break;
        }
    }

    ROS_INFO("Final joint positions were reached? %d", reached);
*/
    ROS_INFO("Trajectory execution complete!");

    // mark the goal as succeeded
    follow_joint_trajectory_as_.setSucceeded();
}

void JointTrajectoryExecutor::publishJointCommand(std::vector<std::string> joint_names, std::vector<double> joint_positions)
{
    // create joint command message
    xbot_msgs::JointCommand joint_command;

    // set time stamp
    joint_command.header.stamp = ros::Time::now();

    // set joint names
    joint_command.name = joint_names;

    // set joint positions
    joint_command.position = std::vector<float>(joint_positions.begin(), joint_positions.end());

    // set control mode unit8t
    joint_command.ctrl_mode = std::vector<uint8_t>(joint_positions.size(), 1);

    // publish joint command
    xbot_joint_command_pub_.publish(joint_command);
}

MoveitXbotBridge::MoveitXbotBridge(ros::NodeHandle nh):
    nh_(nh)
{
    // subscribe to xbot joint states
    xbot_joint_state_sub_ = nh_.subscribe("/xbotcore/joint_states", 1, &MoveitXbotBridge::xbotJointStateCB, this);

    // pubsliher for ros joint states
    ros_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // create the pointer to the current joint state
    current_joint_state_ptr_ = std::make_shared<xbot_msgs::JointState>();

    arm_1_trajectory_executor_ = std::make_shared<JointTrajectoryExecutor>(nh_, arm_1_controller_name_, goal_execution_timeout_, joint_angle_tolerance_, current_joint_state_ptr_);
    arm_2_trajectory_executor_ = std::make_shared<JointTrajectoryExecutor>(nh_, arm_2_controller_name_, goal_execution_timeout_, joint_angle_tolerance_, current_joint_state_ptr_);
    torso_trajectory_executor_ = std::make_shared<JointTrajectoryExecutor>(nh_, torso_controller_name_, goal_execution_timeout_, joint_angle_tolerance_, current_joint_state_ptr_);

    ROS_INFO("MoveIt! Xbot Bridge started!");
}

MoveitXbotBridge::~MoveitXbotBridge()
{
}

void MoveitXbotBridge::xbotJointStateCB(const xbot_msgs::JointState::ConstPtr& msg)
{
    // create joint state message
    sensor_msgs::JointState joint_state;

    // set the joint state pointer
    current_joint_state_ptr_->link_position = msg->link_position;

    // set time stamp
    joint_state.header.stamp = ros::Time::now();

    // set joint names
    joint_state.name = msg->name;

    // set joint positions
    joint_state.position = std::vector<double>(msg->link_position.begin(), msg->link_position.end());

    // publish joint state
    ros_joint_state_pub_.publish(joint_state);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_xbot_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Starting MoveIt! Xbot Bridge...");
    MoveitXbotBridge moveit_xbot_bridge(nh);

    // set the rate hz
    ros::Rate rate(100);

    // spin
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}