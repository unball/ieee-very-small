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
	return(game_state_);
}

/**
 * Updates which team has the ball
 */
void StateEstimator::updateBallState()
{
}

/**
 * Updates the score each time a team makes a goal
 */
void StateEstimator::updateScore()
{
}