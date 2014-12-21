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

/**
 * Updates all state estimations.
 */
void StateEstimator::update()
{
    updateBallState();
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
        ROS_INFO("Ball state: Attack");
    }
    else if (ball_x < -0.25)
    {
        ball_state_ = WorldState::BALL_DEFENSE_FIELD;
        ROS_INFO("Ball state: Defense");
    }
    else
    {
        ball_state_ = WorldState::BALL_MIDDLE_FIELD;
        ROS_INFO("Ball state: Middle");
    }
}

/**
 * Updates the score each time a team makes a goal.
 */
void StateEstimator::updateScore()
{
}