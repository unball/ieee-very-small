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
    float getPredictedOrientation();
    void predictPose();
    void predictOrientation();
    void updatePosition(cv::Point2f measured_pose);
    void updateOrientation(float measured_orientation);
    void restart();

  private:
    cv::Point2f predicted_pose_;
    cv::Point2f predicted_velocity_;
    float predicted_orientation_;
    float delta_theta_;
    float weight_;
};



#endif // UNBALL_VISION_ROBOT_FILTER_H
