/**
 * @file   play.hpp
 * @author Icaro da Costa Mota
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Play class
 *
 * Defines the requisits of any given play. All plays will inherit from Play
 */

#ifndef UNBALL_PLAY_H_
#define UNBALL_PLAY_H_

#define INITIAL_PLAY_STATE 0

class Play
{
  public:
    Play();
    virtual bool run() = 0; // Implement the code to execute the specific play
    
  protected:
    bool robots_action_finished_[6];
    int play_state_;
};

#endif  // UNBALL_PLAY_H_

