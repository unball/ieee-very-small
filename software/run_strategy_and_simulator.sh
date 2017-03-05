#!/bin/bash

gnome-terminal \
--tab -e "roscore" \
--tab -e "rosrun rosbridge_server rosbridge_websocket" \
--tab -e "python strategy/simple_strategy/goalkeeper.py" \
--tab -e "python strategy/simple_strategy/relative_position_converter.py" \
--tab -e "python Communication/control/position_control.py" \
--tab -e "python Communication/control/differential_model.py" \
--tab -e "python Communication/control/speed_converter.py" \
--tab -e " ./simulator/unball_simulator.x86_64" \
