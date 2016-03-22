/**
 * @file   tracked_object.hpp
 * @author Gabriel Naves da Silva
 * @date   22/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  TrackedObject class
 */

#include <unball/vision/tracked_object.hpp>

TrackedObject::TrackedObject()
{
    name_ = "TrackedObject_BaseClass";
}

bool TrackedObject::isName(std::string name)
{
    return name == name_;
}
