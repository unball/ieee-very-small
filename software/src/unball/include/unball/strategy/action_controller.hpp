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
 * Defines robots actions for strategy
 */

#ifndef UNBALL_ACTION_CONTROLLER_H_
#define UNBALL_ACTION_CONTROLLER_H_

#include <ros/ros.h>

#include <unball/strategy/robot.hpp>

class ActionController
{
  public:
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

