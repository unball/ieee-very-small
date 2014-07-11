#include <unball/strategy/play_formation_2.hpp>
#include <ros/ros.h>
#include <unball/strategy/action_controller.hpp> // ActionControler action_controller;

PlayFormation2::PlayFormation2() : Play()
{
	play_name_ = "PLAY FORMATION 2";
}

void PlayFormation2::initialRosMessage()
{
	ROS_INFO("PLAY FORMATION 2 RUN");
}

void PlayFormation2::setUnfinishedActions()
{
    if (robots_action_finished_[3] && robots_action_finished_[4])
    {
        robots_action_finished_[3] = false;
        robots_action_finished_[4] = false;
        ++play_state_;
    }
}

bool PlayFormation2::act()
{
    switch (play_state_)
    {
        // force initial stop (in case the last play was interrupted)
        case 0:
            ROS_INFO("PLAY FORMATION 2 STATE 0");
            action_controller.stop(3);
            action_controller.stop(4);
            break;
        case 1:
            ROS_INFO("PLAY FORMATION 2 STATE 1");
            action_controller.goTo(3, -0.15, 0.30);
            action_controller.goTo(4, -0.15, -0.30);
            break;
        case 2:
            ROS_INFO("PLAY FORMATION 2 STATE 2");
            action_controller.lookAt(3, 0, 0);
            action_controller.lookAt(4, 0, 0);
            break;
        default:
            ROS_INFO("PLAY FORMATION 2 FINISHED");
            play_state_ = 0; // Reseting play state for the next time the play is called
            return true;
    }
    
    return false;	
}
