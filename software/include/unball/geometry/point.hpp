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
    Point();
    Point(float x, float y);

    void set(float x, float y);
    void set(const Point& point);

    float getX() const;
    float getY() const;

    void movePointWithAngle(float distance, float angle);

    float findAngle(const Point& point) const;
    bool equals(const Point& point);
    float distance(const Point& point);

    Point operator+(const Point& rhs) const;
    Point operator-(const Point& rhs) const;
    Point operator*(const float rhs) const;
    Point operator/(const float rhs) const;
    bool operator==(const Point& rhs) const;

  private:
    float x, y;
};

#endif // POINT_H_

