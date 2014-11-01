/**
 * @file   strategy.cpp
 * @author Matheus Vieira Portela
 * @author Icaro da Costa Mota
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Strategy class
 *
 * Implements strategy for robots
 */

#include <unball/strategy/strategy.hpp>

/**
 * Strategy global object
 */
Strategy strategy;

Strategy::Strategy()
{
    play_controller_.pushPlay(new PlayFormation2());
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
            state_estimator_.setGameState(WorldState::GAME_RUNNING);
            break;
        case 'p': case 'P':
            ROS_INFO("Setting game state: PAUSED");
            state_estimator_.setGameState(WorldState::GAME_PAUSED);
            break;
        case 'a': case 'A':
            ROS_INFO("Setting game state: ABORT");
            state_estimator_.setGameState(WorldState::GAME_ABORTED);
            play_controller_.abortPlay();
            break;
        case '1':
            ROS_INFO("Formation 1");
            play_controller_.abortPlay();
            play_controller_.pushPlay(new PlayFormation1());
            break;
        case '2':
            ROS_INFO("Formation 2");
            play_controller_.abortPlay();
            play_controller_.pushPlay(new PlayFormation2());
            break;
    }
}

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    if (state_estimator_.getGameState() != WorldState::GAME_PAUSED)
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

/**
 * Choose the best strategy state depending on the state of the game
 */
void Strategy::chooseStrategyState()
{

}

/**
 * Find the robot that is closest to the ball
 */
void Strategy::findRobotClosestToBall()
{

}