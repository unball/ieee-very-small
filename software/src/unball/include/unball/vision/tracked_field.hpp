/**
 * @file   tracked_field.hpp
 * @author Matheus Vieira Portela
 * @date   31/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Field class definition for tracker
 */

#ifndef UNBALL_VISION_TRACKED_FIELD_H_
#define UNBALL_VISION_TRACKED_FIELD_H_

#include <unball/vision/tracked_object.hpp>

class TrackedField : public TrackedObject
{
  public:
    int exponentialMovingAvg(int old_value, int new_value);
    void updatePosition(cv::Point position);
    void updateBoundingRect(cv::Rect bounding_rect);
    void drawMarker(cv::Mat &frame);

  private:
    static const float AVG_CONSTANT;
    static const int CIRCLE_RADIUS;
    static const cv::Scalar RECTANGLE_COLOR;
    static const cv::Scalar CIRCLE_COLOR;
};

#endif // UNBALL_VISION_TRACKED_FIELD_H_