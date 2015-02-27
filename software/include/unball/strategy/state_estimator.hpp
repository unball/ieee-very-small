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

#include <cmath>

#include <ros/ros.h>

#include <unball/strategy/robot.hpp>
#include <unball/strategy/ball.hpp>
#include <unball/utils/math.hpp>

namespace WorldState
{
    enum BallState
    {
        BALL_ATTACK_FIELD,
        BALL_DEFENSE_FIELD,
        BALL_MIDDLE_FIELD
    };
    
    enum BallPossessionState
    {
        BALL_POSSESSION_OURS,
        BALL_POSSESSION_THEIRS,
        BALL_POSSESSION_NONE
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
    WorldState::BallState getBallState();
    WorldState::BallPossessionState getBallPossessionState();
    bool hasStateChanged();
    void update();

  private:
  	void updateBallState();
    int closestRobotToBall();
    void updateBallPossessionState();
    void updateScore();

    int score;
    WorldState::GameState game_state_;
    WorldState::BallState ball_state_;
    WorldState::BallPossessionState ball_possession_state_;
    WorldState::BallPossessionState prev_ball_possession_state_;
};

#endif  // UNBALL_STATE_ESTIMATOR_H_