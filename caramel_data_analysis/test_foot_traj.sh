#!/bin/bash

# $1: bag name
# $2: X
# $3: Y
# $4: remote user
# $5: remote IP

ros2 launch caramel_data_analysis foot_traj_test.launch.py x:=$2 y:=$3 bag_name:=$1
trfbags $1 $4 $5