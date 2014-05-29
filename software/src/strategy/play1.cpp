#include "play1.hpp"
#include <ros/ros.h>
#include "action_controller.hpp" // ActionControler action_controller;

bool Play1::run()
{
    ROS_INFO("PLAY 1 RUN");
    
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
            this->play_state_ = 0; // Reseting play state for the next time the play is called
            return true;
    }
    
    return false;
}
