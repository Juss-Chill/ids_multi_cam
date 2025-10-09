#!/bin/bash

sudo -E bash -c "source /opt/ros/noetic/setup.bash && \
source ./../../../../../devel/setup.bash && \
rosrun rosradar radar_parse.py"
