#include <unball/vision/ball_filter.hpp>

BallFilter::BallFilter()
{
    predicted_velocity_ = cv::Point2f(0, 0);
    predicted_pose_ = cv::Point2f(320, 240);
    weight_ = 0.3;
}

cv::Point2f BallFilter::getPredictedPose()
{
    return predicted_pose_;
}

void BallFilter::predict()
{
    predicted_pose_ += predicted_velocity_*1.25;
}

void BallFilter::update(cv::Point2f measured_pose)
{
    cv::Point2f previous_pose = predicted_pose_;
    cv::Point2f pose_discrepancy;

    predicted_pose_ = weight_ * predicted_pose_ +  (1 - weight_) * measured_pose;
    predicted_velocity_ = (predicted_pose_ - previous_pose);
}  

void BallFilter::resetFilter()
{
    predicted_velocity_ = cv::Point2f(0,0);
    predicted_pose_ = cv::Point2f(320, 240);
}

void BallFilter::resetLastPose(cv::Point2f last_pose)
{
    predicted_velocity_ = cv::Point2f(0,0);
    predicted_pose_ = cv::Point2f(last_pose);
}