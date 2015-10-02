/**
 * @file   ball_tracker.hpp
 * @author Matheus Vieira Portela
 * @author Manoel Vieira Coelho Neto
 * @date   30/09/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of ball tracker class
 */

#ifndef UNBALL_VISION_BALL_TRACKER_H_
#define UNBALL_VISION_BALL_TRACKER_H_

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class BallTracker
{
  public:
    cv::Point getBallPose();
    void track(cv::Mat &rgb_segmented_image);
    
  private:  
    cv::Point ball_position_;
};

#endif // UNBALL_VISION_BALL_TRACKER_H_