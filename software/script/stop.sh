#!/bin/bash
#########################################################################
# Script for stopping all ROS processes.
#
# Author: Matheus Vieira Portela
# Date: 14/07/2014
#########################################################################

echo "Killing rosout..."
killall rosout
echo "done"

echo "Killing rosmaster..."
killall rosmaster
echo "done"

echo "Killing roscore..."
killall roscore
echo "done"

echo "All ROS processes are killed"
