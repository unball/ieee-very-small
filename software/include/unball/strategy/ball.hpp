/**
 * @file   ball.hpp
 * @author Matheus Vieira Portela
 * @date   01/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Ball class
 *
 * Defines ball methods for strategy.
 * Ball uses the singleton pattern, since there is only a single ball in the game. In this pattern, only one instance of
 * the Ball class can exist and it can only be accessed through the getInstance() method, which deals with instantiation.
 */

#ifndef UNBALL_BALL_H_
#define UNBALL_BALL_H_

#include <cmath>

#include <ros/ros.h>

#include <unball/geometry/point.hpp>

class Ball
{
  public:
    static Ball& getInstance();

    Point getPos();
    float getX();
    float getY();
    float getVelX();
    float getVelY();
    float getVelAbs();
    float getVelAngle();
    
    void setPosition(float x, float y);
    void setVel(float vel_x, float vel_y);
    void setVelAbs(float vel_abs);
    void setVelAngle(float vel_angle);
    
    void update(float x, float y);
    
  protected:
    Ball();

  private:
    static Ball *instance; // singleton instance
    Point pos_, vel_;
};

#endif  // UNBALL_BALL_H_
