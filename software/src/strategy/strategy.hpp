/**
 * @file   strategy.hpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Strategy class
 *
 * Defines strategy for robots
 */

#ifndef UNBALL_STRATEGY_H_
#define UNBALL_STRATEGY_H_

class Strategy
{
  public:
    void run();
    
    void setRobotLocation(float location, int robot_number);
    float getRobotVelocity(int robot_number);
    
  private:
    float robots_location_[6];
    float robots_velocities_[6];
};

#endif  // UNBALL_STRATEGY_H_
