/**
 * @file   play.cpp
 * @author Icaro da Costa Mota
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  play class
 *
 * Implements the methods that all plays must have
 */

#include "play.hpp"

Play::Play()
{
    for (int i = 0; i < 6; ++i)
        this->robots_action_finished_[i] = false;
    
    this->initPlayState();
}

void Play::initPlayState()
{
    this->play_state_ = INITIAL_PLAY_STATE;
}

void Play::mutexLock(bool &mutex)
{
    mutex = false;
}

void Play::mutexUnlock(bool &mutex)
{
    mutex = true;
}
