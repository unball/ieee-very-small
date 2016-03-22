/**
 * @file   tracked_object.hpp
 * @author Gabriel Naves da Silva
 * @date   22/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  TrackedObject class
 *
 * The TrackedObject class defines the attributes and functionality
 * common to all objects that require tracking, like the robots and the ball.
 */

#ifndef UNBALL_TRACKED_OBJECT_H_
#define UNBALL_TRACKED_OBJECT_H_

#include <string>

#include <opencv2/opencv.hpp>

class TrackedObject
{
  public:
    TrackedObject();
    virtual ~TrackedObject() = 0;

    virtual void runTracking() = 0;

    bool isName(std::string name);

  protected:
    std::string name_;
};

#endif // UNBALL_TRACKED_OBJECT_H_
