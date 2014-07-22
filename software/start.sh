#!/bin/bash
#########################################################################
# Script for launching all ROS processes required for running the UnBall
# Soccer Team code.
#
# Author: Matheus Vieira Portela
# Date: 13/07/2014
#########################################################################

# Check whether roscore is running. If not, run roscore in the background.
if ps -A | grep roscore > /dev/null
then
    echo "roscore is running"
else
    echo "launching roscore"
    roscore&
fi

# Check whether rosmaster was launch (it takes a while)
while !(ps -A | grep rosmaster > /dev/null)
do
    echo "waiting for rosmaster"
    sleep 1
done
echo "rosmaster is running"

# Check whether rosout was launch (it takes a while)
while !(ps -A | grep rosout > /dev/null)
do
    echo "waiting for rosout"
    sleep 1
done
echo "rosout is running"
