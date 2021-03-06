/**
 * @file   player.hpp
 * @author Icaro da Costa Mota
 * @date   08/09/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_PLAYER_H_
#define UNBALL_PLAYER_H_

#include <memory>

#include <unball/utils/vector.hpp>

#include <unball/strategy/ball.hpp>
#include <unball/strategy/robot.hpp>
 #include <unball/strategy/goals.hpp>
#include <unball/strategy/attractive_potential_field.hpp>
#include <unball/strategy/parallel_potential_field.hpp>
#include <unball/strategy/perpendicular_potential_field.hpp>
#include <unball/strategy/repulsive_potential_field.hpp>
#include <unball/strategy/selective_potential_field.hpp>
#include <unball/strategy/tangential_potential_field.hpp>

enum player_behaviour
{
    INITIAL_GOALKEEPER,
    GOALKEEPER,
    GOALKEEPER_KICKER,
    REGULAR_PLAYER,
    ASSISTENT_PLAYER,
    KICKER_PLAYER
};

class Player
{
  public:
  	~Player();
  	virtual void buildPotentialFields(int robot_number) = 0;
  	void clearPotentialFields();
  	Vector calculateResultantForce(int robot_number);

  	player_behaviour getBehaviour();
  protected:
  	std::vector<PotentialField*> potential_fields_;

  	player_behaviour behaviour_;
};

#endif  // UNBALL_PLAYER_H_