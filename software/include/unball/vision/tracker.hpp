/**
 * @file   tracker.hpp
 * @author Gabriel Naves da Silva
 * @date   22/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Tracker class
 *
 * The tracker class manages the objects that are being tracked.
 */

#ifndef UNBALL_TRACKER_H_
#define UNBALL_TRACKER_H_

#include <vector>
#include <memory>

#include <unball/vision/tracked_object.hpp>

class Tracker
{
  public:
    static Tracker& getInstance();

    void runTracking();

  private:
    std::vector<std::shared_ptr<TrackedObject>> tracked_objects_;
};

#endif // UNBALL_TRACKER_H_
