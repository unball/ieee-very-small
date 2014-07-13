/**
 * @file   strategy.cpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Strategy class
 *
 * Implements strategy for robots
 */

#include <unball/strategy/strategy.hpp>
#include <ros/ros.h>

/**
 * Strategy global object
 */
Strategy strategy;

Strategy::Strategy()
{
    play_controller_.pushPlay(PLAY_1);
    setGameState(GAME_RUNNING);
}

void Strategy::setGameState(GameState game_state)
{
    game_state_ = game_state;
}

/**
 * Receive keyboard input and set the proper game state.
 * @param key Input key.
 */
void Strategy::receiveKeyboardInput(char key)
{
    switch (key)
    {
        case 'r': case 'R':
            ROS_INFO("Setting game state: RUNNING");
            setGameState(GAME_RUNNING);
            break;
        case 'p': case 'P':
            ROS_INFO("Setting game state: PAUSED");
            setGameState(GAME_PAUSED);
            break;
        case 'a': case 'A':
            ROS_INFO("Setting game state: ABORT");
            setGameState(GAME_ABORTED);
            play_controller_.abortPlay();
            break;
        case '1':
            ROS_INFO("Formation 1");
            play_controller_.abortPlay();
            play_controller_.pushPlay(PLAY_FORMATION_1);
            break;
        case '2':
            ROS_INFO("Formation 2");
            play_controller_.abortPlay();
            play_controller_.pushPlay(PLAY_FORMATION_2);
            break;
    }
}

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    if (game_state_ != GAME_PAUSED)
    {
        choosePlay();
        play_controller_.run();
    }
}

/**
 * Choose the best play for the current state of the game.
 */
void Strategy::choosePlay()
{
    
}
