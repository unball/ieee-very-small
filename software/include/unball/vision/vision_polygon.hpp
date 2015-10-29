/**
 * @file   vision_polygon.hpp
 * @author Gabriel Naves da Silva
 * @date   29/10/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the polygon class for the vision system.
 */

#ifndef UNBALL_VISION_POLYGON_H_
#define UNBALL_VISION_POLYGON_H_

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class VisionPolygon
{
  public:
    void setPoints(const std::vector<cv::Point2f> &rgb_points);

    bool isPointInside(cv::Point2f point);

  private:
    bool isPointToTheRightOfSegment(cv::Point2f seg_start, cv::Point2f seg_end, cv::Point2f point);

    std::vector<cv::Point2f> points_;
};

#endif // UNBALL_VISION_POLYGON_H_
