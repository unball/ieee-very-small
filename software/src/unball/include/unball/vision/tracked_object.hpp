/**
 * @file   tracked_object.hpp
 * @author Matheus Vieira Portela
 * @date   29/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tracked object definition file for vision module
 */

#ifndef UNBALL_VISION_TRACKED_OBJECT_H_
#define UNBALL_VISION_TRACKED_OBJECT_H_

#include <opencv2/opencv.hpp>

class TrackedObject
{
  public:
    virtual void updatePosition(cv::Point position) = 0;
    virtual void updateBoundingRect(cv::Rect bounding_rect) = 0;
    virtual void drawMarker(cv::Mat &frame) = 0;

  protected:
    cv::Point position_;
    cv::Rect bounding_rect_;
};

#endif // UNBALL_VISION_TRACKED_OBJECT_H_