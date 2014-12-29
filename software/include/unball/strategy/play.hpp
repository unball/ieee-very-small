/**
 * @file   play.hpp
 * @author Icaro da Costa Mota
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play class
 *
 * Abstract class that defines the required methods of any given play. All plays must inherit from Play.
 * Uses the State Pattern to implement the run() method.
 */

#ifndef UNBALL_PLAY_H_
#define UNBALL_PLAY_H_

#include <ros/ros.h>

#include <unball/strategy/action_controller.hpp>
#include <unball/strategy/ball.hpp>
#include <unball/strategy/robot.hpp>

class Play
{
  public:
    Play();
    bool run();
    std::string getPlayName();
    bool hasFinished();
    
  protected:
    void updateStates();
    virtual void act() = 0; // Method to implement robots actions according to the Play's objective.

    int play_state_[6];
    int num_states_;
    std::string play_name_;
};

#endif  // UNBALL_PLAY_H_

