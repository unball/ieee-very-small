#include <unball/vision/robot_filter.hpp>

RobotFilter::RobotFilter()
{
    predicted_velocity_ = cv::Point2f(0, 0);
    predicted_pose_ = cv::Point2f(0, 0);
    weight_ = 0.5;
}

cv::Point2f RobotFilter::getPredictedPose()
{
    return predicted_pose_;
}

float RobotFilter::getPredictedOrientation()
{
    return predicted_orientation_;
}

void RobotFilter::predictPose()
{
    predicted_pose_ += predicted_velocity_ * 2;
}

void RobotFilter::predictOrientation()
{
    predicted_orientation_ += delta_theta_ * 1.5;
}

void RobotFilter::updatePosition(cv::Point2f measured_pose)
{
    cv::Point2f previous_pose = predicted_pose_;
    predicted_pose_ = weight_ * predicted_pose_ +  (1 - weight_) * measured_pose;
    predicted_velocity_ = (predicted_pose_ - previous_pose);
}

void RobotFilter::restart()
{
    predicted_velocity_ = cv::Point2f(0, 0);
    predicted_pose_ = cv::Point2f(0, 0);
    weight_ = 0.5;
}

void RobotFilter::updateOrientation(float measured_orientation)
{
    float previous_orientation = predicted_orientation_;

    predicted_orientation_ = weight_ * predicted_orientation_ + (1 - weight_) * measured_orientation;
    delta_theta_ = predicted_orientation_ - previous_orientation;
}
