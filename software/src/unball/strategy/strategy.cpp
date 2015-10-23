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

Strategy::Strategy() {}

/**
 * Receive keyboard input and set the proper game state.
 * @param key Input key.
 */
void Strategy::receiveKeyboardInput(char key)
{
    switch (key)
    {
        case 'r': case 'R':
        	ResumeGame();
        	break;
        case 'p': case 'P':
        	PauseGame();
            break;
    }
}

/*
* Pauses the actions of all robots.
* TODO (mota.icaro@gmail.com): Pause the ball in the simulator
*/
void Strategy::PauseGame()
{
	ROS_INFO("[Strategy] Keyboard input: Setting game state PAUSED");
	state_estimator_.setGameState(WorldState::GAME_PAUSED);
	
	for (int i=0;i<6;i++)
		trajectory_controller_.stopRobot(i);
}

void Strategy::ResumeGame()
{
	ROS_INFO("[Strategy] Keyboard input: Setting game state RUNNING");
	state_estimator_.setGameState(WorldState::GAME_RUNNING);   
}

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    if (state_estimator_.getGameState() != WorldState::GAME_PAUSED)
    {
		state_estimator_.update();
        updatePlayers();
        trajectory_controller_.run();
    }
}

void Strategy::updatePlayers()
{
    
}