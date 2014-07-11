/**
 * @file   ball.hpp
 * @author Matheus Vieira Portela
 * @date   01/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Ball class
 *
 * Defines ball methods for strategy
 */

#ifndef UNBALL_BALL_H_
#define UNBALL_BALL_H_

class Ball
{
  public:
    Ball();
  
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
    
  private:
    float x_;
    float y_;
    float vel_x_;
    float vel_y_;
    float vel_abs_;
    float vel_angle_;
};

extern Ball ball;

#endif  // UNBALL_BALL_H_
