
#include <unball/strategy/play1.hpp>
#include <ros/ros.h>
#include <unball/strategy/action_controller.hpp> // ActionControler action_controller;

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
    if (robots_action_finished_[3] && robots_action_finished_[4])
    {
        robots_action_finished_[3] = false;
        robots_action_finished_[4] = false;
        ++play_state_;
    }
}

bool Play1::act()
{
    switch (play_state_)
    {
		// force initial stop (in case the last play was interrupted)
        case 0:
            ROS_INFO("PLAY 1 STATE 0");
            action_controller.stop(3);
            action_controller.stop(4);
        case 1:
            ROS_INFO("PLAY 1 STATE 1");
            action_controller.goTo(3, -0.15, 0.15);
            action_controller.goTo(4, 0.30, 0.15);
            break;
        case 2:
            ROS_INFO("PLAY 1 STATE 2");
            action_controller.goTo(3, -0.40, 0.2);
            action_controller.goTo(4, -0.60, 0.2);
            break;
        default:
            ROS_INFO("PLAY 1 FINISHED");
            play_state_ = 0; // Reseting play state for the next time the play is called
            return true;
    }
    
    return false;
}
