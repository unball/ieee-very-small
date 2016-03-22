/**
 * @file   tracker.cpp
 * @author Gabriel Naves da Silva
 * @date   22/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tracker class
 */

#include <unball/vision/tracker.hpp>

Tracker& Tracker::getInstance()
{
    static Tracker tracker;
    return tracker;
}

void Tracker::runTracking()
{
    for(auto object : tracked_objects_)
        object->runTracking();
}
