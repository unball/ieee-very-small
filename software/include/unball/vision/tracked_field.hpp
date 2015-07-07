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

#include <string>
#include <cmath>

#include <ros/ros.h>

#include <unball/vision/tracked_object.hpp>

#define BAD_CONFIG 1

class TrackedField : public TrackedObject
{
  public:
    TrackedField();
    ~TrackedField();

    void loadConfig();
    void loadTrackingMode();

    int exponentialMovingAvg(int old_value, int new_value);
    void updatePosition(cv::Point position);
    void updateTrackingWindow(cv::Rect bounding_rect);
    void checkFieldStabilization(cv::Point old_position, cv::Point new_position);

    void trackWithRGB(cv::Mat &rgb_frame);
    void trackWithDepth(cv::Mat &depth_frame);
    void track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame);

    void draw(cv::Mat &frame);

    bool isFieldStable();
    cv::Point getFieldDimensions();

  private:
    std::string tracking_mode_;
    int field_stabilization_frame_;
    int field_center_discrepancy_;
    int field_stabilization_counter_;
    bool is_field_stable_;

    static const float AVG_CONSTANT;
    static const int CIRCLE_RADIUS;
    static const cv::Scalar RECTANGLE_COLOR;
    static const cv::Scalar CIRCLE_COLOR;
};

#endif // UNBALL_VISION_TRACKED_FIELD_H_
