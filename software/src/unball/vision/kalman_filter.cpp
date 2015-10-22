/**
 * @file   kalman_filter.cpp
 * @author Matheus Vieira Portela
 * @author Manoel Vieira Coelho Neto
 * @date   02/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Implementation of Kalman filter class
 */

#include <unball/vision/kalman_filter.hpp>

KalmanFilter::KalmanFilter()
{
    kalman_filter_.init(4, 2, 0, CV_32F);
    kalman_filter_.transitionMatrix = cv::Mat_<float>(4, 4, CV_32F) << 1, 0, 1, 0,
                                                                       0, 1, 0, 1,
                                                                       0, 0, 1, 0,
                                                                       0, 0, 0, 1;
    kalman_filter_.statePre.at<float>(0) = 0;
    kalman_filter_.statePre.at<float>(1) = 0;
    kalman_filter_.statePre.at<float>(2) = 0;
    kalman_filter_.statePre.at<float>(3) = 0;
    cv::setIdentity(kalman_filter_.measurementMatrix);
    cv::setIdentity(kalman_filter_.processNoiseCov, cv::Scalar::all(100));
    cv::setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar::all(100));
    cv::setIdentity(kalman_filter_.errorCovPost, cv::Scalar::all(100));
}

cv::Point2f KalmanFilter::getEstimatedPose()
{
    return estimated_pose_;
}

void KalmanFilter::update(cv::Point2f measured_pose)
{
    ROS_ERROR("measured_pose: %f, %f", measured_pose.x, measured_pose.y);
    cv::Mat_<float> mat_measured_pose(2, 1, CV_32F);

    mat_measured_pose.at<float>(0) = measured_pose.x;
    mat_measured_pose.at<float>(1) = measured_pose.y;

    cv::Mat_<float> corrected_pose = kalman_filter_.correct(mat_measured_pose);
    estimated_pose_.x = corrected_pose.at<float>(0);
    estimated_pose_.y = corrected_pose.at<float>(1);
}
 
void KalmanFilter::predict()
{
    float ticks = (float) cv::getTickCount();
    float dt  =  (ticks - pre_ticks_)/cv::getTickFrequency();
    pre_ticks_ = ticks;
    kalman_filter_.transitionMatrix.at<float>(2) = dt;
    kalman_filter_.transitionMatrix.at<float>(7) = dt;

    cv::Mat_<float> predicted_pose = kalman_filter_.predict();

    // kalman_filter_.statePre.copyTo(kalman_filter_.statePost);
    // kalman_filter_.errorCovPre.copyTo(kalman_filter_.errorCovPost);

    estimated_pose_.x = predicted_pose.at<float>(0);
    estimated_pose_.y = predicted_pose.at<float>(1);
}

void KalmanFilter::updateWithoutMeasurement()
{
    kalman_filter_.statePre.copyTo(kalman_filter_.statePost);
    kalman_filter_.errorCovPre.copyTo(kalman_filter_.errorCovPost);
}