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

Strategy strategy;

/**
 * Strategy instance.
 */
Strategy* Strategy::instance = NULL;

Strategy& Strategy::getInstance()
{
    if (instance == NULL)
        instance = new Strategy();

    return *instance;
}

Strategy::Strategy()
{
    play_controller_.pushPlay(new NoPlay());
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
            ROS_INFO("[Strategy] Keyboard input: Setting game state RUNNING");
            state_estimator_.setGameState(WorldState::GAME_RUNNING);
            break;
        case 'p': case 'P':
            ROS_INFO("[Strategy] Keyboard input: Setting game state PAUSED");
            state_estimator_.setGameState(WorldState::GAME_PAUSED);
            break;
        case 'a': case 'A':
            ROS_INFO("[Strategy] Keyboard input: Abort play");
            play_controller_.abortPlay();
            break;
        case '1':
            ROS_INFO("[Strategy] Keyboard input: Play Formation 1");
            play_controller_.abortPlay();
            play_controller_.pushPlay(new PlayFormation1());
            break;
        case '2':
            ROS_INFO("[Strategy] Keyboard input: Play Formation 2");
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
        state_estimator_.update();
        // choosePlay();
        // play_controller_.run();
        trajectory_controller_.run();
    }
}

/**
 * Choose the best play for the current state of the game.
 */
void Strategy::choosePlay()
{
    WorldState::BallState ball_state = state_estimator_.getBallState();
    WorldState::BallPossessionState ball_possession_state = state_estimator_.getBallPossessionState();
    
    // Only evaluate plays when there is a change in the state estimation
    if (state_estimator_.hasStateChanged())
    {
        // Abort any on-going play and replace it with the new play
        if (ball_possession_state == WorldState::BALL_POSSESSION_NONE)
        {
            play_controller_.abortPlay();
            play_controller_.pushPlay(new PlayFormation2());
        }
        else if (ball_possession_state == WorldState::BALL_POSSESSION_THEIRS)
        {
            play_controller_.abortPlay();
            play_controller_.pushPlay(new PlayFormation1());
        }
    }
}

/**
 * Choose the best strategy state depending on the state of the game
 */
void Strategy::chooseStrategyState()
{

}