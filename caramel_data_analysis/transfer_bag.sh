#!/bin/bash

# $1: bag name
# $2: remote user
# $3: remote IP

scp -r /home/ubuntu/bm_ws/src/bir-black-mouth/caramel_data_analysis/bags/$1 $2@$3:/home/$2/workspaces/black_mouth_humble/bags