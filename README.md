# iit-repair-ros-pkg

### To use this package simply clone the repositroy and build it with 

``` catkin build ```

### To visualize RePair on RViz and play with its joints:

``` roslaunch repair_urdf repair_full_slider.launch ```

### To launch RePair on RViz without the joint_state_publisher_gui:

``` roslaunch repair_urdf repair_full.launch ```

### To view the robot in Gazebo:

``` roslaunch repair_gazebo repair_gazebo.launch ```

You can ignore the following error messages, the model uses position controllers while p gains are only needed for effort controllers

``` [ERROR] [1675347973.116238028]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/x_joint ```

### To control the simulated robot with moveit:

``` roslaunch repair_gazebo bringup_moveit.launch ``` 

This will automaitcally launch the gazebo simulation as well as rviz with the motion planing tool.
You can ignore the following error messages, the model uses position controllers while p gains are only needed for effort controllers

``` [ERROR] [1675347973.116238028]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/x_joint ```

The hands are currently just placeholders, they have no functionality and no collision


