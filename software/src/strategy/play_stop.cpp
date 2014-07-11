#include "play_stop.hpp"
#include <ros/ros.h>
#include "action_controller.hpp" // ActionControler action_controller;

PlayStop::PlayStop()
{
	play_name_ = "PLAY STOP";
}

void PlayStop::initialRosMessage()
{
	ROS_INFO("PLAY STOP RUN");
}

void PlayStop::setUnfinishedActions() {}

bool PlayStop::act()
{
	for (int i = 0; i < 6; ++i)
		action_controller.stop(i);
    return true;
}
