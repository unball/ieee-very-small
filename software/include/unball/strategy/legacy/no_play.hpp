/**
 * @file   play1.hpp
 * @author Icaro da Costa Mota
 * @date   4/08/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Empty play necessary for PlayController
 */

#ifndef UNBALL_NO_PLAY_H_
#define UNBALL_NO_PLAY_H_

#include <ros/ros.h>

#include <unball/strategy/legacy/play.hpp>
#include <unball/strategy/legacy/action_controller.hpp>

class NoPlay : public Play
{
  public:
    NoPlay();
    
  private:
  	void defineRobotNumbers();
    void act();
};

#endif  // UNBALL_NO_PLAY_H_