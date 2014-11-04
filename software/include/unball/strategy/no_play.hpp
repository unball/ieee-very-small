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

#include <unball/strategy/play.hpp>
#include <unball/strategy/action_controller.hpp>

class NoPlay : public Play
{
  public:
    NoPlay();
  private:
    void setUnfinishedActions();
    bool act();
};

#endif  // UNBALL_NO_PLAY_H_