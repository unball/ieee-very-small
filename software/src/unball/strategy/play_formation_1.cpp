/**
 * @file   play.cpp
 * @author Icaro da Costa Mota
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play formation 1 class
 */

#include <unball/strategy/play_formation_1.hpp>

PlayFormation1::PlayFormation1() : Play()
{
    play_name_ = "PLAY FORMATION 1";
}

/**
 * Sets the actions of both robots that are not the goalkeeper (3 and 4) to false.
 */
void PlayFormation1::setUnfinishedActions()
{
    if (robots_action_finished_[3])
    {
        robots_action_finished_[3] = false;
        ++play_state_[3];
    }
    if (robots_action_finished_[4])
    {
        robots_action_finished_[4] = false;
        ++play_state_[4];
    }
}

/**
 * Stop the action of the robots that are the goalkeeper.
 * Move robot 3 to a defensive position.
 * Move robot 4 to an ofensive position.
 * Rotates them.
 */
bool PlayFormation1::act()
{
    for (int i=0;i<6;i++)
    {
        switch (play_state_[i])
        {
            // force initial stop (in case the last play was interrupted)
            case 0:
                ROS_INFO("PLAY FORMATION 1 STATE 0");
                actState0(i);
                break;
            case 1:
                ROS_INFO("PLAY FORMATION 1 STATE 1");
                actState1(i);
                break;
            case 2:
                ROS_INFO("PLAY FORMATION 1 STATE 2");
                actState2(i);
                break;
            default:;
                ROS_INFO("PLAY FORMATION 1 FINISHED");
                play_state_[i] = 0; // Reseting play state for the next time the play is called
                return true;
        }
    }
    
    return false;
}

void PlayFormation1::actState0(int robot)
{
    if (robot == 3 or robot == 4)
        action_controller.stop(robot);
}

void PlayFormation1::actState1(int robot)
{
    int x,y;
    switch (robot)
    {
        case 3:
            x = -0.30;
            y = 0;
            break;
        case 4:
            x = 0.30;
            y = 0;
            break;
        default:
            return;
    }
    action_controller.goTo(robot, x, y);
}

void PlayFormation1::actState2(int robot)
{
    if (robot == 3 or robot == 4)
        action_controller.lookAt(robot, 0, 0);    
}
