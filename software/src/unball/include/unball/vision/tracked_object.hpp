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

#include <unball/vision/tracked_object_type.hpp>

class TrackedObject
{
  public:
    TrackedObject();
    ~TrackedObject();
    
    virtual void init(cv::Rect &tracking_window);
    virtual void track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame);
    virtual void draw(cv::Mat &frame);

  protected:
    TrackedObjectType type_;
    cv::Point position_;
    float orientation_;
    cv::Rect tracking_window_;
};

#endif // UNBALL_VISION_TRACKED_OBJECT_H_