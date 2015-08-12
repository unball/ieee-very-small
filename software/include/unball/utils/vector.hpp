/**
 * @file   vector.hpp
 * @author Matheus Vieira Portela
 * @date   27/06/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Two-dimensional vector class.
 */

#ifndef UNBALL_VECTOR_H_
#define UNBALL_VECTOR_H_

#include <cmath>
#include <iostream>
#include <string>

#include <ros/ros.h>

class Vector
{
  public:
    // Empty constructor.
    Vector();

    // Initializes with pre-defined x and y values.
    Vector(float x, float y);

    // Copy constructor.
    Vector(const Vector& other);

    // Compound assignment operators.
    Vector& operator+=(const Vector &rhs);
    Vector& operator-=(const Vector &rhs);
    Vector& operator*=(const Vector &rhs);
    Vector& operator*=(float rhs);
    Vector& operator/=(const Vector &rhs);
    Vector& operator/=(float rhs);

    // Binary arithmetic operators.
    const Vector operator+(const Vector &rhs) const;
    const Vector operator-(const Vector &rhs) const;
    const Vector operator*(const Vector &rhs) const;
    const Vector operator*(float rhs) const;
    const Vector operator/(const Vector &rhs) const;
    const Vector operator/(float rhs) const;

    // Comparison operators.
    bool operator==(const Vector &rhs) const;
    bool operator!=(const Vector &rhs) const;

    // Getters and setters
    float getX() const;
    void setX(float x);

    float getY() const;
    void setY(float y);

    // Calculates the magnitude, also known as norm or absolute value.
    float getMagnitude() const;

    // Calculates the direction (in radians).
    float getDirection() const;

    void set(float x, float y);
    void set(Vector vector);

    // Sets vector coordinates given magnitude and direction (in radians).
    void setPolar(float magnitude, float direction);

    // Adds two vectors in Cartesian coordinates.
    void add(Vector vector);

    // Subtracts two vectors in Cartesian coordinates.
    void subtract(Vector vector);

    // Multiplies vector coordinates by a scalar value.
    void multiply(float scalar);

    // Divides vector coordinates by a scalar value, except zero.
    void divide(float scalar);

    // Multiplies vector coordinates by -1.
    void negate();

    // Rotates vector by an angle (in radians) couterclockwise.
    void rotate(float angle);

    // Adjusts the x, y coordinates such that the vector norm is 1.
    void normalize();

    // Projects the vector onto a given direction.
    void project(Vector direction);

    // Limits vector magnitude to the maximum given value.
    void saturate(float maximum);

    // Calculates the distance to another vector.
    float calculateDistance(Vector vector) const;

    // Converts to string for printing purposes.
    std::string toString() const;

  protected:
    // 2-D (x, y) coordinates
    float x_;
    float y_;
};

std::ostream& operator<<(std::ostream& os, const Vector& vector);

#endif  // UNBALL_VECTOR_H_
