/**
 * @file   penalty_kicker.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   08/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_PENALTY_KICKER_H_
#define UNBALL_PENALTY_KICKER_H_

#include <unball/strategy/player.hpp>

class PenaltyKicker : public Player
{
  public:
  	void buildPotentialFields(int robot_number);
};

#endif  // UNBALL_PENALTY_KICKER_H_