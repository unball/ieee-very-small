/**
 * @file   trajectory_controller.hpp
 * @author Matheus Vieira Portela
 * @date   03/08/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Control robots trajectory by applying potential fields.
 */

#ifndef UNBALL_TRAJECTORY_CONTROLLER_H_
#define UNBALL_TRAJECTORY_CONTROLLER_H_

#include <memory>

#include <unball/utils/vector.hpp>

#include <unball/strategy/ball.hpp>
#include <unball/strategy/robot.hpp>
#include <unball/strategy/potential_field.hpp>
#include <unball/strategy/attractive_potential_field.hpp>
#include <unball/strategy/parallel_potential_field.hpp>
#include <unball/strategy/perpendicular_potential_field.hpp>
#include <unball/strategy/repulsive_potential_field.hpp>
#include <unball/strategy/selective_potential_field.hpp>
#include <unball/strategy/tangential_potential_field.hpp>
#include <unball/strategy/regular_player.hpp>
#include <unball/strategy/goalkeeper.hpp>
#include <unball/strategy/initial_goalkeeper.hpp>
#include <unball/strategy/kicker_player.hpp>

class TrajectoryController
{
  public:
    TrajectoryController();
    ~TrajectoryController();
    void run();
    void stopRobot(int robot_number);

    void updatePlayer(int robot_number, player_behaviour behaviour);

    Player* getPlayer(int robot_number);
  private:
    Player* player_[3];

    std::vector<PotentialField*> potential_fields_;
    float angle_error_d_;

    void controlRobot(int robot_number, Vector force);
    void turn(int robot_number, float angle);
    void move(int robot_number, float distance);

    bool isInBallRange(int robot_number);
};

#endif  // UNBALL_TRAJECTORY_CONTROLLER_H_