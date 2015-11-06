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
        case 'k': case 'K':
            GoalKick();
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

	for (int i=0;i<3;i++)
		trajectory_controller_.stopRobot(i);
}

void Strategy::ResumeGame()
{
	ROS_INFO("[Strategy] Keyboard input: Setting game state RUNNING");
	state_estimator_.setGameState(WorldState::GAME_RUNNING);
}

void Strategy::GoalKick()
{
    trajectory_controller_.updatePlayer(2,GOALKEEPER_KICKER);
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
    else
    {
      for (int i=0;i<3;i++)
        trajectory_controller_.stopRobot(i);
    }
}

/**
 * REFACTOR: put this inside each player. maybe a method like: player_behaviour shouldChangeTo();
 */
void Strategy::updatePlayers()
{
    for (int i=0;i<3;i++)
    {
       if (trajectory_controller_.getPlayer(i)->getBehaviour() == INITIAL_GOALKEEPER)
       {
            Vector goalkeeper_pos = Vector(robot[i].getPos().getX(),robot[i].getPos().getY());

            if (goalkeeper_pos.calculateDistance(Goals::getInstance().friendly_goal_) < 0.2)
                trajectory_controller_.updatePlayer(i,GOALKEEPER);
       }
       else if (trajectory_controller_.getPlayer(i)->getBehaviour() == GOALKEEPER)
       {
            //if (Goals::getInstance().isBallInFriendlyGoalArea())
            //    trajectory_controller_.updatePlayer(i,GOALKEEPER_KICKER);
       }
       else if (trajectory_controller_.getPlayer(i)->getBehaviour() == GOALKEEPER_KICKER)
       {
       //     if (not Goals::getInstance().isBallInFriendlyGoalArea())
       //         trajectory_controller_.updatePlayer(i,INITIAL_GOALKEEPER);
       }
       else if (trajectory_controller_.getPlayer(i)->getBehaviour() == KICKER_PLAYER)
       {
            // if (not hasBall(i))
            //     trajectory_controller_.updatePlayer(i,ASSISTENT_PLAYER);
       }
       /*else if (trajectory_controller_.getPlayer(i)->getBehaviour() == ASSISTENT_PLAYER)
       {
            if (isThere(KICKER_PLAYER))
            {
                setKickerForAssistent(i);
            }
            else
            {
                if (hasBall(i))
                    trajectory_controller_.updatePlayer(i,KICKER_PLAYER);
                else
                    trajectory_controller_.updatePlayer(i,ASSISTENT_PLAYER);
            }
       }*/
    }
}

bool Strategy::isThere(player_behaviour behaviour)
{
    return find(behaviour) != -1;
}

int Strategy::find(player_behaviour behaviour)
{
    for (int i=0; i<3; i++)
        if (trajectory_controller_.getPlayer(i)->getBehaviour() == behaviour)
            return i;
    return -1;
}

void Strategy::setKickerForAssistent(int assistent)
{
    delete trajectory_controller_.getPlayer(assistent);
    Player *p = trajectory_controller_.getPlayer(assistent);
    p = new AssistentPlayer(find(KICKER_PLAYER));
}

bool Strategy::hasBall(int robot_number)
{
    Vector ball_pos(Ball::getInstance().getX(),Ball::getInstance().getY());
    Vector robot_pos(robot[robot_number].getX(),robot[robot_number].getY());

    if ((robot_pos - ball_pos).getMagnitude() > 0.5)
        return false;

    float direction = (ball_pos - Goals::getInstance().opponent_goal_).getDirection();

    Vector difference = robot_pos - ball_pos;
    return (fabs(difference.getDirection() - direction) <= M_PI/2);
}
