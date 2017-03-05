#!/bin/bash

gnome-terminal \
--tab -e "roscore" \
--tab -e "python strategy/simple_strategy/go_to_ball.py" \
--tab -e "python Communication/control/position_control.py" \
--tab -e "python strategy/simple_strategy/relative_position_converter.py" \
--tab -e "python Communication/control/differential_model.py" \
--tab -e "python Communication/control/speed_converter.py" \
--tab -e "roslaunch communication run_communication.launch" \