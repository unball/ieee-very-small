/**
 * @file   action_controller.hpp
 * @author Matheus Vieira Portela
 * @author Icaro da Costa Mota
 * @date   25/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Action class
 *
 * ActionController is a singleton.
 * Actions are the basic movement unit for our robots, hence, every play must execute actions sequentially.
 * There is a single action controller that controls each robot asynchronously.
 * Every action must be implemented by a boolean method, which returns "true" only when the required action has
 * finished.
 */

#ifndef UNBALL_ACTION_CONTROLLER_H_
#define UNBALL_ACTION_CONTROLLER_H_

#include <ros/ros.h>

#include <unball/strategy/robot.hpp>
#include <unball/utils/math.hpp>

class ActionController
{
  public:
    static ActionController& getInstance();
    void run();
    bool hasRobotFinished(int robot_number);
    void stop(int robot_number);
    bool executeStop(int robot_number);
    void move(int robot_number, float distance);
    bool executeMove(int robot_number);
    void lookAt(int robot_number, float x, float y);
    bool executeLookAt(int robot_number);
    void goTo(int robot_number, float x, float y);
    bool executeGoTo(int robot_number);
    
  private:
    // singleton instance
    static ActionController *instance;

    // Move attributes
    Point move_initial_[6];
    float move_distance_[6];
    
    // Look at attributes
    Point look_at_[6];
    
    // Go to attributes
    Point go_to_[6];
    float go_to_error_dist_i_[6];
    float go_to_error_dist_d_[6];
    float go_to_error_ang_i_[6];
    float go_to_error_ang_d_[6];
};

extern ActionController action_controller;

#endif  // UNBALL_ACTION_CONTROLLER_H_

