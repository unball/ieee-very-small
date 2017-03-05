#!/bin/bash

gnome-terminal \
--tab -e "roslaunch vision run_vision.launch" \
--tab -e "rosrun unball measurement_system_node" \
--tab -e "./simulator/unball_simulator.x86_64" \
--tab -e "rosrun rosbridge_server rosbridge_websocket" \