/**
 * @file   point.cpp
 * @author Icaro da Costa Mota
 * @author Gabriel Naves da Silva
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Point class
 *
 * Implements geometry methods for dealing with points in 2D
 */

#include <unball/geometry/point.hpp>

Point::Point()
{
    x = 0, y = 0;
}

Point::Point(float x, float y)
{
    set(x, y);
}

void Point::set(float x, float y)
{
    this->x = x;
    this->y = y;
}

void Point::set(const Point& point)
{
    x = point.x;
    y = point.y;
}

float Point::getX() const { return x; }
float Point::getY() const { return y; }

/**
* Moves the point to a new position using a distance and an angle
* given in radians.
*/
void Point::movePointWithAngle(float distance, float angle)
{
    x = x + cos(angle) * distance;
    y = y + sin(angle) * distance;
}

/**
* Finds the angle in radians of the semiline defined by
* two points (the one at hand and the one given as parameter)
* relative to the positive x axis.
* @param point The other point that defines the semiline
*/
float Point::findAngle(const Point& point) const
{
    if (point.y == y)
        return (point.x >= x ? 0 : M_PI);
    if (point.x == x)
        return (point.y >= y ? M_PI/2 : 3*M_PI/2);
    if (point.x > x && point.y > y)
        return (atan((point.y - y)/(point.x-x)));
    if (point.x > x && point.y < y)
        return (2*M_PI - atan((y - point.y)/(point.x-x)));
    if (point.x < x && point.y < y)
        return (M_PI + atan((y - point.y)/(x-point.x)));
    if (point.x < x && point.y > y)
        return (M_PI - atan((point.y - y)/(x-point.x)));
    return 0;
}

/**
* Returns true if the distance between both points is lower than
* a tolerance, and false otherwise.
*/
bool Point::equals(const Point& point)
{
    return distance(point) < 10 ? true : false;
}

float Point::distance(const Point& point)
{
    return (sqrt(pow(point.x-x, 2) + pow(point.y-y, 2)));
}

Point Point::operator+(const Point& rhs) const
{
    return Point(x + rhs.x, y + rhs.y);
}

Point Point::operator-(const Point& rhs) const
{
    return Point(x - rhs.x, y - rhs.y);
}

Point Point::operator*(const float rhs) const
{
    return Point(x * rhs, y * rhs);
}

Point Point::operator/(const float rhs) const
{
    //beware: does not treat divisions by 0
    return Point(x / rhs, y / rhs);
}

bool Point::operator==(const Point& rhs) const
{
    return x == rhs.x and y == rhs.y ? true : false;
}
