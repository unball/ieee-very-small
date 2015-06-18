/**
 * @file   keyboard_input.cpp
 * @author Izabella Thais Oliveira Gomes
 * @author Icaro da Costa Mota
 * @date   18/06/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief keyboard_input class
 *
 * Read keyboard input
 */

#include <unball/strategy/keyboard_input.hpp>


/**
 * Print the character read on the terminal while the roslauch is running.
 */
 void KeyboardInput::printTerminal()
 {
 	input = 'i';
 	ROS_ERROR("Received key: %c", input);
 }