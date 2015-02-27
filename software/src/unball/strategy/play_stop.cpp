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

PlayStop::PlayStop() : Play()
{
    play_name_ = "PLAY STOP";
    num_states_ = 1;
}

void PlayStop::defineRobotNumbers() {}

/**
 * Stops all actions from all robots
 */
void PlayStop::act()
{
    for (int i = 0; i < 6; ++i)
    {
        ROS_INFO("[PlayStop] Robot %d state %d", i, play_state_[i]);
        ActionController::getInstance().stop(i);
    }
}
