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

#define INITIAL_PLAY_STATE 0

class Play
{
  public:
    Play();
    bool run();
    std::string getPlayName();
    
  protected:
    // [matheus.v.portela@gmail.com - 04/11/2014] Why is this method virtual?
    // For those robots who have not finished their actions yet, set action_finished_ to false
    virtual void setUnfinishedActions() = 0;
    
    // Method to implement robots actions according to the Play's objective.
    // Must return true when the play is finished.
    virtual bool act() = 0;

    void initialRosMessage();
    int findRobotClosestToBall(std::vector<int> index);

    bool robots_action_finished_[6];
    int play_state_[6];
    std::string play_name_;
    
  private:
    void finishRobotAction(int i);
};

#endif  // UNBALL_PLAY_H_

