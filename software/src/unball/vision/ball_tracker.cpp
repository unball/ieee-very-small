#include <unball/vision/ball_tracker.hpp>

cv::Point BallTracker::getBallPosition()
{
    return ball_position_;
}

void BallTracker::track(cv::Mat &rgb_frame, cv::Mat &rgb_segmented_image)
{
    cv::Moments moments;
    std::vector< std::vector<cv::Point> > countours;

    cv::findContours(rgb_segmented_image, countours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    moments = cv::moments(countours[0], true);
    ball_position_.x = moments.m10/moments.m00;
    ball_position_.y = moments.m01/moments.m00;
    ROS_INFO("Ball x %d", ball_position_.x);
    ROS_INFO("Ball y %d", ball_position_.y);
}
