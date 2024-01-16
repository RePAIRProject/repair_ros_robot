# Guides for Dummies (to run the Pick & Place demo with perception)

Assuming installation is finished:
## Running the demo

#### Terminals
Open (at least) two terminals, go to `~/repair_robot_ws` and source what you need `source devel/setup.zsh` (also activate conda environment `conda activate ros_robot`).

#### Launch the setup for the scene
In one terminal, run `roslaunch repair_gazebo bringup_moveit.launch` (this launches Rviz and Gazebo).

#### Launch the pick & place demo (with perception)
In another terminal, when the scene is ready, run `roslaunch repair_interface moveit_test.launch side:=right gazebo:=false` (this starts the pick & place demo).


## Point clouds
point clouds are processed in `repair_interface/scripts/moveit_test.py` around line 262:
```python
if use_pyrealsense:
    pcd = get_point_cloud_from_real_rs(debug)
else:
    pcd = get_point_cloud_from_ros(debug)
```

## Spawning fragments
The fragments spawned are listed in `repair_gazebo/launch/repair_gazebo.launch`, as:
```xml
<!-- spawn the first fragment -->    
<arg name="fragment1_model" default="$(find repair_urdf)/sdf/frag2/model.sdf"/>
<node name="fragment1_description1" pkg="gazebo_ros" type="spawn_model" 
args="-file $(arg fragment1_model) -sdf -model frag2 -x 0.0 -y 0.0 -z 1.5" />
```  

## Creating new fragments
Fragments are in `repair_ros_robot/repair_urdf/sdf`

Currently, if you want to add a new fragment, you have to do it manually:
- Copy one of the fragment folders "frag", frag2" or frag3"
- Change the name of the folder, let's say "frag4"
- Adjust the name in the "model.config" to "frag_4"
- Replace the stl in the mesh folder and rename it to "frag_4"
- Change the name in the model.sdf to "frag4". Also adjust the path of the visual shape to "<uri>model://frag4/meshes/frag_4.stl</uri>" and adjust the values of the collision shape to the bounding box values of the fragment. Alternatively you can use the visual shape as collision shape (the same stl), but that is computationally more expensive and will slow down the Gazebo simulation.

## Notes
Pysdf is used to load the sdf file into Gazebo. Then a  Gazebo plugin publishes the fragment to to rviz. This means you won't see the fragment shape in rviz, only its pose.
