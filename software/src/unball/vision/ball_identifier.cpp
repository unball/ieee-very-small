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

#include <unball/vision/ball_identifier.hpp>

const cv::Scalar BallIdentifier::CIRCLE_COLOR_(0, 0, 255);

BallIdentifier::BallIdentifier(MeasurementConversion *mc){
    to_metric_ = mc;
    ball_pose_ = cv::Point2f(320, 240);
    counter_ = 0;
}

cv::Point2f BallIdentifier::getBallPose()
{
    cv::Point2f metric_position = to_metric_->convertToMetric(ball_pose_);
    return metric_position;
}

std::vector<cv::Point> BallIdentifier::findLargerBlob(std::vector< std::vector<cv::Point> > contours)
{
    std::vector<cv::Point> comparison_contour;
    comparison_contour = contours[0];
    for (int i = 0; i < contours.size() - 1; ++i)
    {
        if (cv::contourArea(comparison_contour) < cv::contourArea(contours[i+1]))
            comparison_contour = contours[i+1];
    }
    return comparison_contour;
}

void BallIdentifier::track(cv::Mat &rgb_frame, cv::Mat &rgb_segmented_image)
{
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> larger_contour;
    cv::Moments moments;

    cv::findContours(rgb_segmented_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    if (contours.size() != 0)
    {
        larger_contour = findLargerBlob(contours);
        moments = cv::moments(larger_contour, true);
        ball_pose_ = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
        tracker_.update(ball_pose_);
    }
    else
    {
        counter_++;
        tracker_.predict();
        if (counter_ > 15)
            tracker_.resetLastPose(ball_pose_);
        if (isOutOfLimits())
            tracker_.resetFilter();
        
        ball_pose_ = tracker_.getPredictedPose();

    }


    cv::circle(rgb_frame, ball_pose_, 10, CIRCLE_COLOR_);
};

bool BallIdentifier::isOutOfLimits()
{
    if(ball_pose_.x < 0 or ball_pose_.y < 0 or
       ball_pose_.x > 640 or ball_pose_.y > 480)
        return true;
    else 
        return false;
}