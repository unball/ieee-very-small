/**
 * @file   kicker_player.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   22/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_KICKER_PLAYER_H_
#define UNBALL_KICKER_PLAYER_H_

#include <unball/strategy/player.hpp>

class KickerPlayer : public Player
{
  public:
  	KickerPlayer();
  	void buildPotentialFields(int robot_number);
  
  private:
  	void findTarget();

  	static float const BALL_RANGE_ = 0.3;

  	bool isInBallRange(int robot_number);
  	bool opponentGoalkeeperIsInGoalRange(int opponent_goalkeeper);

  	float target_;
  	Vector kick_target_;
};

#endif  // UNBALL_KICKER_PLAYER_H_