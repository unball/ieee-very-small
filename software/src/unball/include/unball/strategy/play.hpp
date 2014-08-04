/**
 * @file   play.hpp
 * @author Icaro da Costa Mota
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play class
 *
 * Defines the requisits of any given play. All plays will inherit from Play
 * Uses the State Pattern to implement the run() method.
 */

#ifndef UNBALL_PLAY_H_
#define UNBALL_PLAY_H_

#define INITIAL_PLAY_STATE 0

#include <cmath>

#include <ros/ros.h>

#include <unball/strategy/action_controller.hpp>
#include <unball/strategy/ball.hpp>
#include <unball/strategy/robot.hpp>

class Play
{
  public:
    Play();
    bool run(); // Implement the code to execute the specific play
    std::string getPlayName();
    
  protected:
    //for those robots who have not finished their actions yet, set action_finished_ to false
    virtual void setUnfinishedActions() = 0;
    //moves the robots accoardingly to the play's objective. returns true if the action is done
    virtual bool act() = 0;

    void initialRosMessage();
    int findRobotClosestToBall(std::vector<int> index);

    bool robots_action_finished_[6];
    int play_state_;
    std::string play_name_;
    
  private:
    void finishRobotAction(int i);
};

#endif  // UNBALL_PLAY_H_

