/**
 * @file   regular_player.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   08/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_IDLE_PLAYER_H_
#define UNBALL_IDLE_PLAYER_H_

#include <unball/strategy/player.hpp>

class IdlePlayer : public Player
{
  public:
  	void buildPotentialFields(int robot_number);

};

#endif  // UNBALL_IDLE_PLAYER_H_