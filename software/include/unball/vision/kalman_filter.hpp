/**
 * @file   kalman_filter.hpp
 * @author Matheus Vieira Portela
 * @author Manoel Vieira Coelho Neto
 * @date   02/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Definition of Kalman filter class
 */
#ifndef UNBALL_VISION_KALMAN_FILTER_H_
#define UNBALL_VISION_KALMAN_FILTER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class KalmanFilter
{
  public:
    KalmanFilter();
    void predict();
    void update(cv::Point2f measured_pose);
    cv::Point2f getEstimatedPose();
    void updateWithoutMeasurement();
  private:
    cv::Point2f estimated_pose_;
    cv::KalmanFilter kalman_filter_;
    float pre_ticks_;
};

#endif