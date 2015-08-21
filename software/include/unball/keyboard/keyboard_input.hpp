/**
 * @file   ball.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   18/06/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Keyboard Input class
 *
 * Read keyboard input
 */

#include <stdio.h>
#include <termios.h>
#include <ros/ros.h>

 class KeyboardInput
{
  public:
  	void printLastChar();
  	int readChar();

  private:
  	int last_char_;
};