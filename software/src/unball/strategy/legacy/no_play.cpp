/**
 * @file   no_play.cpp
 * @author Icaro da Costa Mota
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Empty play class
 */

#include <unball/strategy/legacy/no_play.hpp>

 NoPlay::NoPlay() : Play()
{
    play_name_ = "NO PLAY";
    num_states_ = 0;
}

void NoPlay::defineRobotNumbers() {}

void NoPlay::act() {}