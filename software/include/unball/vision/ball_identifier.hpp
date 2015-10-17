/**
 * @file   ball_tracker.hpp
 * @author Matheus Vieira Portela
 * @author Manoel Vieira Coelho Neto
 * @date   30/09/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Definition of ball tracker class
 */

#ifndef UNBALL_VISION_BALL_TRACKER_H_
#define UNBALL_VISION_BALL_TRACKER_H_

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <unball/vision/ball_tracker.hpp>

#include <unball/vision/measurement_conversion.hpp>
 
class BallIdentifier
{
  public:
    BallIdentifier(MeasurementConversion *mc);
    cv::Point2f getBallPose();
    void track(cv::Mat &rgb_frame, cv::Mat &rgb_segmented_image);
    std::vector<cv::Point> findLargerBlob(std::vector< std::vector<cv::Point> > contours);
     
  private:  
    static const cv::Scalar CIRCLE_COLOR_;
    cv::Point2f ball_pose_;
    BallTracker tracker_;
    MeasurementConversion *to_metric_;
};

#endif // UNBALL_VISION_BALL_TRACKER_H_