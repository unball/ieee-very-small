/**
 * @file   state_estimator.hpp
 * @author Icaro da Costa Mota
 * @date   28/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief StateEstimator class
 *
 * A class that estimates the state of the game based on the data given by the camera
 */

#ifndef UNBALL_STATE_ESTIMATOR_H_
#define UNBALL_STATE_ESTIMATOR_H_

#include <ros/ros.h>

namespace WorldState
{
    enum BallState
    {
        OURS,
        THEIRS
    };
    
    enum GameState
    {
        GAME_RUNNING,
        GAME_PAUSED,
        GAME_STOPPED,
        GAME_ABORTED,
        GAME_FIELD_KICK,
        GAME_GOAL,
        GAME_FINISHED
    };
};

class StateEstimator
{
  public:
  	StateEstimator();
    
    void setGameState(WorldState::GameState game_state);
    WorldState::GameState getGameState();

    int score;  	
  private:
  	void updateBallState();
    void updateScore();

    WorldState::GameState game_state_;
    WorldState::BallState ball_state_;
};

#endif  // UNBALL_STATE_ESTIMATOR_H_