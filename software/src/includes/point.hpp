/**
 * @file   point.hpp
 * @author Icaro da Costa Mota
 * @author Gabriel Naves da Silva
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Point class
 *
 * Defines a point, so you can deal with variables with 2 dimensions
 */

#ifndef POINT_H_
#define POINT_H_

#include <iostream>
#include <cmath>

class Point
{
  public:
    //Methods for setting and getting variables
    Point();
    Point(float x, float y);
    void Set(const Point& point);
    void Set(float x, float y);
    float GetX();
    float GetY();

    //Operations
    void Move(float distance, float angle);
    Point operator+(const Point& point);
    Point operator-(const Point& point);
    Point operator*(float rhs);
    Point operator/(float rhs);

    //Getting information on two points, considering a cartesian space
    float DistanceTo(Point other_point);
    bool IsOnTheLeftOf(Point other_point);
    bool IsOnTheRightOf(Point other_point);
    bool HasSameX(Point other_point);
    bool IsAbove(Point other_point);
    bool IsLowerThan(Point other_point);
    bool HasSameY(Point other_point);
    bool IsEqualTo(Point other_point);
    //TODO(mota.icaro@gmail.com):before doing this method, make sure you are dealing with a cartesian space. 
    //If not, change the above methods
    float Inclination(Point other_point);

  private:
    float x,y;
};

#endif // POINT_H_

