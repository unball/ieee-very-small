/**
 * @file   assistent_player.hpp
 * @author Icaro da Costa Mota
 * @date   24/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_ASSISTENT_PLAYER_H_
#define UNBALL_ASSISTENT_PLAYER_H_

#include <unball/strategy/player.hpp>

class AssistentPlayer : public Player
{
  public:
  	AssistentPlayer();
  	AssistentPlayer(int friendly_kicker);
  	void buildPotentialFields(int robot_number);
  private:
  	void findTarget();
  	bool isInBallRange(int robot_number);

  	int friendly_kicker_;
  	Vector kick_target_;

  	static float const BALL_RANGE_ = 0.5;
};

#endif  // UNBALL_ASSISTENT_PLAYER_H_