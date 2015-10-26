#ifndef UNBALL_VISION_ROBOT_FILTER_H
#define UNBALL_VISION_ROBOT_FILTER_H

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>


class RobotFilter
{
  public:
    RobotFilter();
    cv::Point2f getPredictedPose();
    void predict();
    void update(cv::Point2f measured_pose);
    void restart();
  private:
    cv::Point2f predicted_pose_;
    cv::Point2f predicted_velocity_;
    float weight_;
};



#endif // UNBALL_VISION_ROBOT_FILTER_H
