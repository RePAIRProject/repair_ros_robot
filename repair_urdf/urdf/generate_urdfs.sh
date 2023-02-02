#!/bin/sh

# code snippet to generate the full URDF (RePair + cage). Useful if the bare URDF is needed.
# first build and install the package (meshes, etc.. need to be visible to ROS) and then run the script to generate
# the urdf

xacro repair_full.urdf.xacro is_sliding_wrist:=true gen_coll:=false load_sol:=true use_updated_wrist:=false -o repair_full.urdf


 