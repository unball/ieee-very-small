#include <unball/vision/ball_tracker.hpp>

BallTracker::BallTracker()
{
    predicted_velocity_ = cv::Point2f(0, 0);
    predicted_pose_ = cv::Point2f(0, 0);
    weight_ = 0.5;
}

cv::Point2f BallTracker::getPredictedPose()
{
    return predicted_pose_;
}

void BallTracker::predict()
{
    predicted_pose_ += predicted_velocity_ * 1.07;
}

void BallTracker::update(cv::Point2f measured_pose)
{
    cv::Point2f previous_pose = predicted_pose_;
    predicted_pose_ = weight_ * predicted_pose_ +  (1 - weight_) * measured_pose;
    predicted_velocity_ = (predicted_pose_ - previous_pose);
}
