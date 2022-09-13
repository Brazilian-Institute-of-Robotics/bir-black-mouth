#!/bin/bash

. settings.sh

# DELETE FIRST IF EXIST
docker build -t "${PROJECT}_${ROSDISTRO}:devel" .
