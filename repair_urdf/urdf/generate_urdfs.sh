#!/bin/sh

# code snippet to generate the full URDF (RePair + cage)
# this is useful when it's 
xacro repair_full.urdf.xacro is_sliding_wrist:=true gen_coll:=false load_sol:=true xbot:=true use_updated_wrist:=false -o repair_full.urdf


 