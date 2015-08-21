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

#include <unball/keyboard/keyboard_input.hpp>


/**
 * Print the character read on the terminal while the roslauch is running.
 */
 void KeyboardInput::printLastChar()
 {
 	ROS_ERROR("Received key: %c", last_char_);
 }

 /**
 * Read a char from standard input without blocking the execution of a program
 * Stores the char read into last_char_
 */
int KeyboardInput::readChar()
{
    static struct termios oldt, newt;
    last_char_ = ' ';

    // Save old settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable buffering
    newt.c_lflag &= ~(ICANON);
    //newt.c_lflag &= ~(ICANON|ECHO); // Disable echo to the screen

    // Apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read character (non-blocking)
    last_char_ = getchar();

    // Restore old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return last_char_;
}