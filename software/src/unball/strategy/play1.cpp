/**
 * @file   play1.cpp
 * @author Matheus Vieira Portela
 * @author Icaro da Costa Mota
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play 1 class
 */

#include <unball/strategy/play1.hpp>

Play1::Play1() : Play()
{
    play_name_ = "PLAY 1";
}

/**
 *  In this example play, robots 3 and 4 (the counter starts at 0) should not finish their actions simultaniosly.
 *  If both of them have finished their actions, they go back into acting.
 */
void Play1::setUnfinishedActions()
{
    if (robots_action_finished_[3] and robots_action_finished_[4])
    {
        robots_action_finished_[3] = false;
        robots_action_finished_[4] = false;
        ++play_state_[3];
        ++play_state_[4];
    }
}

bool Play1::act()
{
    for (int i=0; i<6; i++)
    {
        switch (play_state_[i])
        {
            // force initial stop (in case the last play was interrupted)
            case 0:
                ROS_INFO("PLAY 1 STATE 0");
                actState0(i);
            case 1:
                ROS_INFO("PLAY 1 STATE 1");
                actState1(i);
                break;
            case 2:
                ROS_INFO("PLAY 1 STATE 2");
                actState2(i);
                break;
            default:
                ROS_INFO("PLAY 1 FINISHED");
                play_state_[i] = 0; // Reseting play state for the next time the play is called
            return true;
        }
    }
    
    return false;
}

void Play1::actState0(int robot)
{
    if (robot == 3 or robot == 4)
        action_controller.stop(robot);
}

void Play1::actState1(int robot)
{
    int x,y;
    switch (robot)
    {
        case 3:
            x = -0.15;
            y = 0.15;
            break;
        case 4:
            x = 0.30;
            y = 0.15;
            break;
        default:
            return;
    }
    action_controller.goTo(robot, x, y);
}

void Play1::actState2(int robot)
{
    int x,y;
    switch (robot)
    {
        case 3:
            x = -0.40;
            y = 0.2;
            break;
        case 4:
            x = -0.60;
            y = 0.2;
            break;
        default:
            return;
    }
    action_controller.goTo(robot, x, y);
}