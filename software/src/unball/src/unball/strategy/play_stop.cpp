/**
 * @file   play.cpp
 * @author Icaro da Costa Mota
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play stop class
 */

#include <unball/strategy/play_stop.hpp>

PlayStop::PlayStop()
{
    play_name_ = "PLAY STOP";
}

void PlayStop::setUnfinishedActions()
{
}

/**
 * Stops all actions from all robots
 */
bool PlayStop::act()
{
    for (int i = 0; i < 6; ++i)
        action_controller.stop(i);
    return true;
}