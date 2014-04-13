/**
 * @file   robot.hpp
 * @author Matheus Vieira Portela
 * @date   27/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Robot class
 *
 * Defines robots for strategy
 */

#ifndef UNBALL_ROBOT_H_
#define UNBALL_ROBOT_H_

class Robot
{
  public:
    float getX();
    float getY();
    float getTh();
    float getLinVel();
    float getAngVel();
    
    void setX(float x);
    void setY(float y);
    void setTh(float th);
    void setLinVel(float lin_vel);
    void setAngVel(float ang_vel);
    
  private:
    // Location attributes
    float x_;
    float y_;
    float th_;
    
    // Velocity attributes
    float lin_vel_;
    float ang_vel_;
};

#endif  // UNBALL_ROBOT_H_

