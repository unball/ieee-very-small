/**
 * @file   state_estimator.cpp
 * @author Icaro da Costa Mota
 * @date   28/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief StateEstimator class
 *
 * A class that estimates the state of the game based on the data given by the camera
 */

#include <unball/strategy/state_estimator.hpp>

 StateEstimator::StateEstimator()
 {
    setGameState(WorldState::GAME_RUNNING);
    score = 0;
 }

void StateEstimator::setGameState(WorldState::GameState game_state)
{
    game_state_ = game_state;
}

WorldState::GameState StateEstimator::getGameState()
{
    return game_state_;
}

WorldState::BallState StateEstimator::getBallState()
{
    return ball_state_;
}

WorldState::BallPossessionState StateEstimator::getBallPossessionState()
{
    return ball_possession_state_;
}

bool StateEstimator::hasStateChanged()
{
    return (ball_possession_state_ != prev_ball_possession_state_);
}

/**
 * Updates all state estimations.
 */
void StateEstimator::update()
{
    updateBallState();
    updateBallPossessionState();
}

/**
 * Updates where in the field the ball is.
 */
void StateEstimator::updateBallState()
{
    float ball_x = Ball::getInstance().getX();
    float ball_y = Ball::getInstance().getY();

    if (ball_x > 0.25)
    {
        ball_state_ = WorldState::BALL_ATTACK_FIELD;
        ROS_INFO("[StateEstimator] Ball state: Attack");
    }
    else if (ball_x < -0.25)
    {
        ball_state_ = WorldState::BALL_DEFENSE_FIELD;
        ROS_INFO("[StateEstimator] Ball state: Defense");
    }
    else
    {
        ball_state_ = WorldState::BALL_MIDDLE_FIELD;
        ROS_INFO("[StateEstimator] Ball state: Middle");
    }
}

/**
 * Calculate the closest robot to the ball.
 */
int StateEstimator::closestRobotToBall()
{
    float ball_x = Ball::getInstance().getX();
    float ball_y = Ball::getInstance().getY();

    float dist;
    float min_dist = 1000;
    int closest_index;

    for (int i = 0; i < 6; ++i)
    {
        dist = math::calculateDistance(robot[i].getX(), robot[i].getY(), ball_x, ball_y);

        if (dist < min_dist)
        {
            min_dist = dist;
            closest_index = i;
        }
    }

    return closest_index;
}

/**
 * Ball possession may be from our team, from the adversary team, or neither of them.
 * We define that the closest robot to the ball, within an arbitrary 0.2 radius, maintains its possession.
 */
void StateEstimator::updateBallPossessionState()
{
    float ball_x = Ball::getInstance().getX();
    float ball_y = Ball::getInstance().getY();
    int closest_index = closestRobotToBall();
    float dist = math::calculateDistance(robot[closest_index].getX(), robot[closest_index].getY(), ball_x, ball_y);

    if (dist > 0.2)
        closest_index = -1;

    prev_ball_possession_state_ = ball_possession_state_;

    if (closest_index == -1)
    {
        ball_possession_state_ = WorldState::BALL_POSSESSION_NONE;
        ROS_INFO("[StateEstimator] Ball possession: no one");
    }
    else if ((closest_index >= 0) and (closest_index < 3))
    {
        ball_possession_state_ = WorldState::BALL_POSSESSION_THEIRS;
        ROS_INFO("[StateEstimator] Ball possession: theirs");
    }
    else if (closest_index >= 3)
    {
        ball_possession_state_ = WorldState::BALL_POSSESSION_OURS;
        ROS_INFO("[StateEstimator] Ball possession: ours");
    }
}

/**
 * Updates the score each time a team makes a goal.
 */
void StateEstimator::updateScore()
{
}