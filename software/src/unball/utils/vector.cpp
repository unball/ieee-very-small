/**
 * @file   vector.cpp
 * @author Matheus Vieira Portela
 * @date   27/06/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Two-dimensional vector class.
 */

#include <unball/utils/vector.hpp>

Vector::Vector() : x_(0), y_(0) {}

Vector::Vector(float x, float y) : x_(x), y_(y) {}

Vector::Vector(const Vector& other)
{
    x_ = other.getX();
    y_ = other.getY();
}

Vector& Vector::operator+=(const Vector &rhs)
{
    x_ += rhs.getX();
    y_ += rhs.getY();
    return *this;
}

Vector& Vector::operator-=(const Vector &rhs)
{
    x_ -= rhs.getX();
    y_ -= rhs.getY();
    return *this;
}

Vector& Vector::operator*=(const Vector &rhs)
{
    x_ *= rhs.getX();
    y_ *= rhs.getY();
    return *this;
}

Vector& Vector::operator*=(float rhs)
{
    x_ *= rhs;
    y_ *= rhs;
    return *this;
}

Vector& Vector::operator/=(const Vector &rhs)
{
    if (rhs.getX() == 0 || rhs.getY() == 0)
    {
        ROS_ERROR("[Vector] Division by vector with zero component");
    }

    x_ /= rhs.getX();
    y_ /= rhs.getY();
    return *this;
}

Vector& Vector::operator/=(float rhs)
{
    if (rhs == 0)
    {
        ROS_ERROR("[Vector] Division by zero scalar");
    }

    x_ /= rhs;
    y_ /= rhs;
    return *this;
}

const Vector Vector::operator+(const Vector &rhs) const
{
    return Vector(*this) += rhs;
}

const Vector Vector::operator-(const Vector &rhs) const
{
    return Vector(*this) -= rhs;
}

const Vector Vector::operator*(const Vector &rhs) const
{
    return Vector(*this) *= rhs;
}

const Vector Vector::operator*(float rhs) const
{
    return Vector(*this) *= rhs;
}

const Vector Vector::operator/(const Vector &rhs) const
{
    return Vector(*this) /= rhs;
}

const Vector Vector::operator/(float rhs) const
{
    return Vector(*this) /= rhs;
}

bool Vector::operator==(const Vector &rhs) const
{
    bool equalX = x_ == rhs.getX();
    bool equalY = y_ == rhs.getY();
    return (equalX && equalY);
}

bool Vector::operator!=(const Vector &rhs) const
{
    return !(*this == rhs);
}

float Vector::getX() const
{
    return x_;
}

void Vector::setX(float x)
{
    x_ = x;
}

float Vector::getY() const
{
    return y_;
}

void Vector::setY(float y)
{
    y_ = y;
}

float Vector::getMagnitude() const
{
    return hypot(x_, y_);
}

float Vector::getDirection() const
{
    return atan2(y_, x_);
}

void Vector::set(float x, float y)
{
    x_ = x;
    y_ = y;
}

void Vector::set(Vector vector)
{
    x_ = vector.getX();
    y_ = vector.getY();
}

void Vector::setPolar(float magnitude, float direction)
{
    x_ = magnitude*cos(direction);
    y_ = magnitude*sin(direction);
}

void Vector::add(Vector vector)
{
    x_ += vector.getX();
    y_ += vector.getY();
}

void Vector::subtract(Vector vector)
{
    x_ -= vector.getX();
    y_ -= vector.getY();
}

void Vector::multiply(float scalar)
{
    x_ *= scalar;
    y_ *= scalar;
}

void Vector::divide(float scalar)
{
    if (scalar == 0.0)
    {
        ROS_ERROR("[Vector] Division by scalar zero");
    }

    x_ /= scalar;
    y_ /= scalar;
}

void Vector::negate()
{
    multiply(-1);
}

void Vector::rotate(float angle)
{
    float new_x = x_*cos(angle) - y_*sin(angle);
    float new_y = x_*sin(angle) + y_*cos(angle);

    x_ = new_x;
    y_ = new_y;
}

void Vector::normalize()
{
    float magnitude = getMagnitude();

    if (magnitude > 0)
    {
        x_ /= magnitude;
        y_ /= magnitude;
    }
    else
    {
        ROS_WARN("[Vector] Normalizing a zero magnitude vector does not change anything");
    }
}

void Vector::project(Vector direction)
{
    Vector projection = (((*this)*direction) / (direction*direction)) * direction;
    set(projection);
}

void Vector::saturate(float maximum)
{
    if (getMagnitude() > maximum)
    {
        normalize();
        multiply(maximum);
    }
}

float Vector::calculateDistance(Vector vector) const
{
    return hypot(vector.getX() - x_, vector.getY() - y_);
}

std::string Vector::toString() const
{
    char buffer[64];
    sprintf(buffer, "(%f, %f)", x_, y_);
    std::string stringBuffer = buffer;

    return stringBuffer;
}

std::ostream& operator<<(std::ostream& os, const Vector& vector)
{
    os << vector.toString();
    return os;
}