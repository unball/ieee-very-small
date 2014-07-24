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
    if (robots_action_finished_[3] && robots_action_finished_[4])
    {
        robots_action_finished_[3] = false;
        robots_action_finished_[4] = false;
        ++play_state_;
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
    switch (play_state_)
    {
        // force initial stop (in case the last play was interrupted)
        case 0:
            ROS_INFO("PLAY FORMATION 1 STATE 0");
            action_controller.stop(3);
            action_controller.stop(4);
            break;
        case 1:
            ROS_INFO("PLAY FORMATION 1 STATE 1");
            action_controller.goTo(3, -0.30, 0);
            action_controller.goTo(4, 0.30, 0);
            break;
        case 2:
            ROS_INFO("PLAY FORMATION 1 STATE 2");
            action_controller.lookAt(3, 0, 0);
            action_controller.lookAt(4, 0, 0);
            break;
        default:;
            ROS_INFO("PLAY FORMATION 1 FINISHED");
            play_state_ = 0; // Reseting play state for the next time the play is called
            return true;
    }
    
    return false;
}
