#!/bin/bash

# $1: bag name
# $2: use_stabilization
# $3: remote user
# $4: remote IP

ros2 launch caramel_data_analysis stability_velocity_test.launch.py use_stabilization:=$2 bag_name:=$1
. transfer_bag.sh $1 $3 $4