/**
 * @file   tactic.cpp
 * @author Icaro da Costa Mota
 * @date   20/09/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief tactic class
 */

#include <unball/strategy/tactic.hpp>

Tactic::Tactic(int robot_num)
{
    /**
     * robots_action_finished_ is true when the last action has finished. Therefore, it must be initialized with false
     * indicating that no action was completed yet.
     */
    robots_action_finished_[robot_num] = false;
    
    tactic_state_ = INITIAL_TACTIC_STATE;
}

std::string Tactic::getTacticName()
{
    return(tactic_name_);
}

void Tactic::initialRosMessage()
{
    std::string message = tactic_name_ + " RUN";
    ROS_INFO("%s", message.c_str());
}

/**
 * Implements the basic structure the tactics must follow.
 */
bool Tactic::run()
{
    initialRosMessage();
    
    // finish the action of all robots that have ended their actions
    for (int i = 0; i < 6; ++i) 
        finishRobotAction(i);
    // for those who have not finished their actions yet, set action_finished_ to false
    setUnfinishedActions();
    return act(); //return true if action has finished
}

/**
 * Finish the action of the robot if the actionController says it should finish
 */
void Tactic::finishRobotAction(int i)
{
    if (action_controller.hasRobotFinished(i))
        robots_action_finished_[i] = true;
}

/**
 * Finds the robot closest to the ball amongst any number of robots you want to
 * @param a vector of the indexes of the robots you want to compare the distance
 */
int Tactic::findRobotClosestToBall(std::vector<int> index)
{
    float min_distance = 999999;
    int min_index;
    for (int i=0;i<index.size();i++)
    {
        //gets the distance from the robot to the ball
        float distance = robot[index[i]].getPos().distance(Ball::getInstance().getPos());
        //if this distance is less than the distance stored, this is the closest robot
        if (distance<min_distance) 
        {
            min_distance = distance;
            min_index = index[i];
        }
    }
    return(min_index);
}
