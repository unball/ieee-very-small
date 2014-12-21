/**
 * @file   play.cpp
 * @author Icaro da Costa Mota
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play formation 2 class
 *
 * Move both robots to defensive positions.
 */

#include <unball/strategy/play_formation_2.hpp>

PlayFormation2::PlayFormation2() : Play()
{
    play_name_ = "PLAY FORMATION 2";
    num_states_ = 3;
}

void PlayFormation2::act()
{
    for (int i = 0; i < 6; ++i)
    {
        ROS_INFO("[PlayFormation2] Robot %d state %d", i, play_state_[i]);

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
                action_controller.stop(i);
                break;
        }
    }
}

/**
 * Force initial stop in case the last play was interrupted.
 */
void PlayFormation2::actState0(int robot)
{
    if (robot == 3 or robot == 4)
        action_controller.stop(robot);
}

/**
 * Moves both robots to defensive positions.
 */
void PlayFormation2::actState1(int robot)
{
    if (robot == 3)
        action_controller.goTo(3, -0.45, 0.40);
    else if (robot == 4)
        action_controller.goTo(4, -0.45, -0.40);
}

/**
 * Look to the center.
 */
void PlayFormation2::actState2(int robot)
{
    if (robot == 3 or robot == 4)
        action_controller.lookAt(robot, 0, 0);    
}