/**
 * @file   strategy.hpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Strategy class
 *
 * Defines strategy for robots
 */

#ifndef UNBALL_STRATEGY_H_
#define UNBALL_STRATEGY_H_

#include <ros/ros.h>

#include <unball/strategy/play_controller.hpp>

namespace GameState
{
    enum BallState
    {
        OURS,
        THEIRS
    };

    enum StrategyState
    {
        OFFENSIVE,
        DEFENSIVE,
        STALLING,
        VERY_OFFENSIVE
    };
    
    enum GlobalState
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

class Strategy
{
  public:
    Strategy();
    void setGameState(GameState::GlobalState game_state);
    void receiveKeyboardInput(char key);
    void run();
    
  private:
    void choosePlay();
    void updateBallState(GameState::BallState ball_state);
    void chooseStrategyState();
    void updateScore();

    PlayController play_controller_;
    GameState::GlobalState game_state_;
    GameState::BallState ball_state_;
    GameState::StrategyState strategy_state_;
    int score;
};

extern Strategy strategy;

#endif  // UNBALL_STRATEGY_H_
