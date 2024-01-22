# RePAIR Interface

## MoveitClient
  - get current pose
  - move any selected arm to a give pose
  - move both arms to respected given poses
  - move arms to home position
  - Control the gripper (open/close/value)

## RePairInterface for Gazebo simulation
  - Provides three services to interact with the robots
    - /get_current_pose_srv - will return the current pose of both arms
    - /move_arm_to_pose_srv - will move the selected arm to the target pose
    - /move_both_arms_srv - will move both arms to the respected target poses
    - /gripper_command_srv - will allow to control the gripper on either of the arms.
  - Get the fragment pose and move above to that pose
  - Pick the fragment and move up a little

## SoftHand Details
- Version: v1.0_simple
- Multiplier for mimic joints: 1.2
- Multiplier values can be modified in the file: `~/repair_robot_ws/src/repair_ros_robot/SoftHand-Plugin/softhands_description/urdf/v1.0_simple/sh_v1_simple.xacro`
