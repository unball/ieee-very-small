/**
 * @file   play.cpp
 * @author Icaro da Costa Mota
 * @author Matheus Vieira Portela
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play class
 */

#include <unball/strategy/play.hpp>

Play::Play()
{
    for (int i = 0; i < 6; ++i)
         play_state_[i] = 0;
}

std::string Play::getPlayName()
{
    return play_name_;
}

/**
 * Update robot states based on whether they have finished their actions. Also, only update if it has not reached the
 * last play state.
 */
void Play::updateStates()
{
    for (int i = 0; i < 6; ++i)
    {
        if ((action_controller.hasRobotFinished(i)) and (play_state_[i] < num_states_))
            ++play_state_[i];
    }
}

/**
 * Play is finished when all robots are in the last state.
 */
bool Play::hasFinished()
{
    for (int i = 0; i < 6; ++i)
    {
        if (play_state_[i] < num_states_)
            return false;
    }

    ROS_INFO("[%s] Finished", play_name_.c_str());
    return true;
}

/**
 * Implements the basic structure the plays must follow.
 * First, it updates the play state for each robots. Then, execute the robots actions. Finally, it checks whether the
 * play has finished.
 */
bool Play::run()
{
    ROS_INFO("[%s] Run", play_name_.c_str());

    updateStates();
    act();
    return hasFinished();
}