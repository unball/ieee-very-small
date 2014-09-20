/**
 * @file   tactic.hpp
 * @author Icaro da Costa Mota
 * @date   20/09/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tactic class
 *
 * Defines the requisits of any given tactic. All plays will inherit from this
 * Uses the State Pattern to implement the run() method.
 */

#ifndef UNBALL_TACTIC_H_
#define UNBALL_TACTIC_H_

#define INITIAL_TACTIC_STATE 0

#include <cmath>

#include <ros/ros.h>

#include <unball/strategy/action_controller.hpp>
#include <unball/strategy/ball.hpp>
#include <unball/strategy/robot.hpp>

class Tactic
{
  public:
    Tactic(int robot_num);
    bool run(); // Implement the code to execute the specific tactic
    std::string getTacticName();
    
  protected:
    //for those robots who have not finished their actions yet, set action_finished_ to false
    virtual void setUnfinishedActions() = 0;
    //moves the robots accoardingly to the play's objective. returns true if the action is done
    virtual bool act() = 0;

    void initialRosMessage();

    bool robot_action_finished_;
    int robot_num_;
    int tactic_state_;
    std::string tactic_name_;
    
  private:
    void finishRobotAction();
};

#endif  // UNBALL_TACTIC_H_

