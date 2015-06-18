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

 #include <ros/ros.h>

 class KeyboardInput
{
  public:

  	char read();
  	void printTerminal();

  private:
  	
  	char input;

};