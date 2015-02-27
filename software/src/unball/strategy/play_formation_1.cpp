/**
 * @file   play_formation_1.cpp
 * @author Icaro da Costa Mota
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play formation 1 class
 * 
 * Move one robot to a defensive position and the other to an offensive position.
 */

#include <unball/strategy/play_formation_1.hpp>

PlayFormation1::PlayFormation1() : Play()
{
    play_name_ = "PLAY FORMATION 1";
    num_states_ = 3;

    defineRobotNumbers();
}

void PlayFormation1::defineRobotNumbers()
{
    defensive_robot_ = 3;
    offensive_robot_ = 4;
    neutral_robot_ = 5;    
}

void PlayFormation1::act()
{
    for (int i = 0; i < 6; ++i)
    {
        ROS_INFO("[PlayFormation1] Robot %d state %d", i, play_state_[i]);

        switch (play_state_[i])
        {
            case 0:
                actState0(i);
                break;
            case 1:
                actState1(i);
                break;
            case 2:
                actState2(i);
                break;
            default:
                ActionController::getInstance().stop(i);
                break;
        }
    }
}

/**
 * Force initial stop in case the last play was interrupted.
 */
void PlayFormation1::actState0(int robot)
{
    if (robot == defensive_robot_ or robot == offensive_robot_)
        ActionController::getInstance().stop(robot);
}

/**
 * Move robot 3 to a defensive position.
 * Move robot 4 to an offensive position.
 */
void PlayFormation1::actState1(int robot)
{
    if (robot == defensive_robot_)
        ActionController::getInstance().goTo(defensive_robot_, 0.20, 0.0);
    else if (robot == offensive_robot_)
        ActionController::getInstance().goTo(offensive_robot_, -0.20, 0.0);
}

/**
 * Look to the center.
 */
void PlayFormation1::actState2(int robot)
{
    if (robot == defensive_robot_ or robot == offensive_robot_)
        ActionController::getInstance().lookAt(robot, 0, 0);
}