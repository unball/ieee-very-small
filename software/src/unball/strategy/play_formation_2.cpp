/**
 * @file   play.cpp
 * @author Icaro da Costa Mota
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play formation 2 class
 */

#include <unball/strategy/play_formation_2.hpp>

PlayFormation2::PlayFormation2() : Play()
{
    play_name_ = "PLAY FORMATION 2";
}

/**
 * Sets the actions of both robots that are not the goalkeeper (3 and 4) to false.
 */
void PlayFormation2::setUnfinishedActions()
{
    if (robots_action_finished_[3] and robots_action_finished_[4])
    {
        robots_action_finished_[3] = false;
        robots_action_finished_[4] = false;
        ++play_state_[3];
        ++play_state_[4];        
    }
}

/**
 * Stop the current action for both robots that are not the goalkeeper.
 * Moves both robots to defensive positions.
 * Rotates them.
 */
bool PlayFormation2::act()
{
    for (int i = 0; i < 6; ++i)
    {
        switch (play_state_[i])
        {
            // force initial stop (in case the last play was interrupted)
            case 0:
                ROS_INFO("PLAY FORMATION 2 STATE 0");
                actState0(i);
                break;
            case 1:
                ROS_INFO("PLAY FORMATION 2 STATE 1");
                actState1(i);
                break;
            case 2:
                ROS_INFO("PLAY FORMATION 2 STATE 2");
                actState2(i);
                break;
            default:
                ROS_INFO("PLAY FORMATION 2 FINISHED");
                play_state_[i] = 0; // Reseting play state for the next time the play is called
            return true;
        }        
    }
    
    return false;
}

void PlayFormation2::actState0(int robot)
{
    if (robot == 3 or robot == 4)
        action_controller.stop(robot);
}

void PlayFormation2::actState1(int robot)
{
    int x, y;
    
    switch (robot)
    {
        case 3:
            x = -0.15;
            y = 0.30;
            break;
        case 4:
            x = -0.15;
            y = 0.30;
            break;
        default:
            return;
    }

    action_controller.goTo(robot, x, y);
}

void PlayFormation2::actState2(int robot)
{
    if (robot == 3 or robot == 4)
        action_controller.lookAt(robot, 0, 0);    
}