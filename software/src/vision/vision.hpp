/**
 * @file   vision.hpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Vision class
 *
 * Defines computer vision class
 */

#ifndef UNBALL_STRATEGY_H_
#define UNBALL_STRATEGY_H_

#include <cv_bridge/cv_bridge.h>

class Vision
{
  public:
    void run();
    void setCameraFrame(cv_bridge::CvImage camera_frame);
    float getRobotLocation(int robot_index);
    
  private:
    cv_bridge::CvImage camera_frame_;
    float robot_location_[6];
};

#endif  // UNBALL_STRATEGY_H_
