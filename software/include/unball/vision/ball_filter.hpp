#ifndef UNBALL_VISION_BALL_TRACKER_H
#define UNBALL_VISION_BALL_TRACKER_H

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <cmath>


class BallFilter
{
  public:
    BallFilter();
    cv::Point2f getPredictedPose();
    void predict();
    void update(cv::Point2f measured_pose);
  private:
    cv::Point2f predicted_pose_;
    cv::Point2f predicted_velocity_;
    float weight_;
};



#endif