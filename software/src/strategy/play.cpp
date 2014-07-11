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

#include <unball/strategy/play.hpp>

Play::Play()
{
    /**
     * robots_action_finished_ is true when the last action has finished. Therefore, it must be initialized with false
     * indicating that no action was completed yet.
     */
    for (int i = 0; i < 6; ++i)
        this->robots_action_finished_[i] = false;
    
    this->play_state_ = INITIAL_PLAY_STATE;
}
