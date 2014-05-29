#include "play_formation_1.hpp"
#include <ros/ros.h>
#include "action_controller.hpp" // ActionControler action_controller;

bool PlayFormation1::run()
{
    ROS_INFO("PLAY FORMATION 1 RUN");
    
    for (int i = 0; i < 6; ++i)
    {
        if (action_controller.hasRobotFinished(i))
            this->robots_action_finished_[i] = true;
    }
    
    if (this->robots_action_finished_[3] && this->robots_action_finished_[4])
    {
        this->robots_action_finished_[3] = false;
        this->robots_action_finished_[4] = false;
        ++this->play_state_;
    }
    
    switch (this->play_state_)
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
        default:
            ROS_INFO("PLAY FORMATION 1 FINISHED");
            return true;
    }
    
    return false;
}
