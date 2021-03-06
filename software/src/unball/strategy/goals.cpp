/**
 * @file   goals.cpp
 * @author Icaro da Costa Mota
 * @date   23/10/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief
 */

#include <unball/strategy/goals.hpp>

Goals* Goals::instance = NULL;

Goals& Goals::getInstance()
{
    if (instance == NULL)
        instance = new Goals();

    return *instance;
}

void Goals::setGoalkeeperSide(float y)
{
	if (y > 0) 
	{
		friendly_goal_ = Vector(0,0.85);
   		opponent_goal_ = Vector(0,-0.85);
	}
	else
	{
		friendly_goal_ = Vector(0,-0.85);
   		opponent_goal_ = Vector(0,0.85);
	}
}

int Goals::findOpponentGoalkeeper()
{
	for (int i=3; i<6; ++i)
	{
		if (fabs(robot[i].getY()) > 0.55) 
			if (robot[i].getX() > -0.22 and robot[i].getX() < 0.22) 
				return i;
	}
	return 3; //THIS IS A HACK: return ANY so our program will not bug.
}

bool Goals::isBallInFriendlyGoalArea()
{
	float goal_area_height;
	
	Vector ball_pos(Ball::getInstance().getX(),Ball::getInstance().getY());

	if (friendly_goal_.getY() < 0) 
	{
		goal_area_height = friendly_goal_.getY() + 0.25;
		if (ball_pos.getY() > goal_area_height)
			return false;
	}
	else 
	{
		goal_area_height = friendly_goal_.getY() - 0.25;
		if (ball_pos.getY() < goal_area_height)
			return false;
	}

    return (ball_pos.getX() > -0.22 and ball_pos.getX() < 0.22);
}