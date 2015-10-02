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

#include <unball/vision/ball_tracker.hpp>

const cv::Scalar BallTracker::CIRCLE_COLOR_(0, 0, 255);

BallTracker::BallTracker(void){
    ball_pose_ = cv::Point2f(0.0, 0.0);
}

cv::Point2f BallTracker::getBallPose()
{
    return ball_pose_;
}

void BallTracker::track(cv::Mat &rgb_frame, cv::Mat &rgb_segmented_image)
{
    std::vector< std::vector<cv::Point> > contours;
    cv::Moments moments;

    cv::findContours(rgb_segmented_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    filter_.predict();
    
    if (contours.size() == 0)
    {
        cv::circle(rgb_frame, ball_pose_, 10, CIRCLE_COLOR_);
        ROS_ERROR("Ball x %f", ball_pose_.x);
        ROS_ERROR("Ball y %f", ball_pose_.y);
    }
    else
    {
        moments = cv::moments(contours[0], true);
        filter_.update(cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00));
        cv::circle(rgb_frame, ball_pose_, 10, CIRCLE_COLOR_);
    }
    ball_pose_ = filter_.getEstimatedPose();
};