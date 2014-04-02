/**
 * @file   strategy.hpp
 * @author Icaro da Costa Mota
 * @date   31/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Simulation class
 *
 */

#ifndef UNBALL_SIMULATION_H_
#define UNBALL_SIMULATION_H_

#include <vector>
#include "robot.hpp"
#include "math/Vector3.hh"

class Simulation{
  public:
    void Run():
    //TODO: Creates all these methods and make sure these are what you want. make sure these paramethers are right
    //simulation.SetRobotLocation();
    void SetRobotLinearVel(const double &_x, const double &_y, const double &_z);
    void SetRobotAngularVel(const double &_x, const double &_y, const double &_z);
  private:
  
}

#endif  // UNBALL_SIMULATION_H_
