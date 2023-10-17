#ifndef MOVEIT_CLIENT_H
#define MOVEIT_CLIENT_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/callback_queue.h"

using u32    = uint_least32_t; 
using engine = std::mt19937;

namespace rvt = rviz_visual_tools;

class MoveitClient
{
    public:

        MoveitClient(ros::NodeHandle nh);

        virtual ~MoveitClient();

        ros::NodeHandle nh_;

        std::string PLANNING_GROUP_ARM_1 = "arm_1";
        std::string PLANNING_GROUP_ARM_2 = "arm_2";

        /* std::string PLANNING_GROUP_HAND_1 = "hand_1"; */
        /* std::string PLANNING_GROUP_HAND_2 = "hand_2"; */

        std::string PLANNING_GROUP_BOTH_ARMS = "both_arms";

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_1;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_2;

        /* std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_hand_1; */
        /* std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_hand_2; */

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_both_arms;

        moveit::planning_interface::MoveGroupInterface::Plan arm_1_plan;
        moveit::planning_interface::MoveGroupInterface::Plan arm_2_plan;

        /* moveit::planning_interface::MoveGroupInterface::Plan hand_1_plan; */
        /* moveit::planning_interface::MoveGroupInterface::Plan hand_2_plan; */

        moveit::planning_interface::MoveGroupInterface::Plan both_arms_plan;

        // visualize_pose subscriber
        ros::Subscriber visualize_pose_sub_;

        // goal_pose subscriber
        ros::Subscriber execute_target_pose_sub_;

        // enums for arm and hand
        enum class ARM {ARM_1, ARM_2};
        /* enum HAND {HAND_1, HAND_2}; */

        enum HAND_STATE {OPEN, CLOSE, VALUE};
        
        std::vector<geometry_msgs::PoseStamped> getCurrentPose();

        bool moveToHome(enum ARM arm);

        /* bool controlHand(enum HAND hand, enum HAND_STATE state, double value=0.0); */

        void visualizePoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void executePoseCB(const std_msgs::Bool::ConstPtr& msg);

        bool sendPoseToSingleArm(geometry_msgs::PoseStamped pose, enum ARM arm);

        bool sendPoseToBothArms(geometry_msgs::PoseStamped arm_1_pose, geometry_msgs::PoseStamped arm_2_pose);
        
        bool visitPose(geometry_msgs::PoseStamped arm_1_pose, geometry_msgs::PoseStamped arm_2_pose, float jump_thresh=5.0f);

        void initMoveitClient();

        void convertQuatToRPY(geometry_msgs::PoseStamped pose, double& roll, double& pitch, double& yaw);

        void getArmMoveGroupAndPlan(enum ARM arm, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group, 
                                    moveit::planning_interface::MoveGroupInterface::Plan& plan);
};

#endif // MOVEIT_CLIENT_H
