/**
 * @file   test_angles.hpp
 * @author Icaro Mota
 * @author Izabella Gomes
 * @date   01/09/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Unit test of the vector class
 */

#ifndef UNBALL_TEST_ANGLES_H_
#define UNBALL_TEST_ANGLES_H_

#include <cmath>
#include <iostream>
#include <string>

#include <unball/utils/vector.hpp>
#include <ros/ros.h>

class TestAngles
{
  public:
    bool run();
  private:
    bool testZero();

    //positives
    bool testPI_4();
    bool testPI_2();
    bool test3PI_4();
    bool testPI();

    //negatives
    bool testNegativePI_4();
    bool testNegativePI_2();
    bool testNegative3PI_4();
    bool testNegativePI();

    bool verifyAngle(Vector origin, Vector destination, float tested_angle);
};

#endif  // UNBALL_TEST_ANGLES_H_
