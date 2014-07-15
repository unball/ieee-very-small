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

#include <unball/strategy/play_controller.hpp>

enum GameState
{
    GAME_RUNNING,
    GAME_PAUSED,
    GAME_STOPPED,
    GAME_ABORTED,
    GAME_FIELD_KICK,
    GAME_GOAL,
    GAME_FINISHED,
};

class Strategy
{
  public:
    Strategy();
    void setGameState(GameState game_state);
    void receiveKeyboardInput(char key);
    void run();
    void choosePlay();
    
  private:
    PlayController play_controller_;
    GameState game_state_;
};

extern Strategy strategy;

#endif  // UNBALL_STRATEGY_H_
