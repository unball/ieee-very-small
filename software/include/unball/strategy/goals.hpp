/**
 * @file   goals.hpp
 * @author Icaro da Costa Mota
 * @date   23/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_GOALS_H_
#define UNBALL_GOALS_H_

#include <ros/ros.h>
#include <cmath>

#include <unball/utils/vector.hpp>
#include <unball/strategy/robot.hpp>
#include <unball/strategy/ball.hpp>

class Goals
{
  public:
    static Goals& getInstance();

  	Vector friendly_goal_;
    Vector opponent_goal_;

	void setGoalkeeperSide(float x);
	int findOpponentGoalkeeper();
	bool isBallInFriendlyGoalArea();
  private:
    static Goals *instance;
};

#endif  // UNBALL_GOALS_H_