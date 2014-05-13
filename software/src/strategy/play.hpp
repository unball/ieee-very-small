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

#ifndef UNBALL_ROBOT_H_
#define UNBALL_ROBOT_H_

class Play
}
  public:
    Play();
    virtual void run() = 0;
  private:
    ActionController action_controller_;
    bool robots_action_finished_[6];
    int play_state_;
    void mutexLock(bool &mutex);
    void mutexUnlock(bool &mutex);
};

#endif  // UNBALL_ROBOT_H_

