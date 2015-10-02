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

class KalmanFilter
{
  public:
    KalmanFilter();
    cv::Point2f getEstimatedPose();
    void update(cv::Point2f measured_pose);
    void predict();

  private:
    cv::Point2f estimated_pose_;
    cv::KalmanFilter kalman_filter_;
    float pre_ticks_;
};

#endif