/**
 * @file   homography.hpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the computer vision homography class
 */

#include <unball/vision/homography.hpp>

Homography::Homography()
{
    dst_points_.push_back(cv::Point2f(0.0,0.0));
    dst_points_.push_back(cv::Point2f(320.0,0.0));
    dst_points_.push_back(cv::Point2f(640.0,0.0));

    dst_points_.push_back(cv::Point2f(0.0,480.0));
    dst_points_.push_back(cv::Point2f(320.0,480.0));
    dst_points_.push_back(cv::Point2f(640.0,480.0));
}

void Homography::loadConfig()
{
    bool calibrate;
    ros::param::get("/vision/homography/calibrate", calibrate);
    current_step_ = (calibrate ? CALIBRATION : RECTIFICATION);
    
    if (current_step_ == RECTIFICATION)
    {
        // somehow loads the calibration matrix
    }
}

void Homography::run(std::vector<cv::Point2f> rgb_points, std::vector<cv::Point2f> depth_points)
{
    switch (current_step_)
    {
        case CALIBRATION:
            calcCalibrationMat(depth_points, rgb_points);
            break;
        case RECTIFICATION:
            calcHomographyMat(rgb_points);
            break;
        default:
            break;
    }
}

void Homography::calcCalibrationMat(std::vector<cv::Point2f> src_points, std::vector<cv::Point2f> dst_points)
{
    if (src_points.size() != 6 or dst_points.size() != 6)
    {
        ROS_WARN("6 points are needed for calibration.");
        return;
    }

    cv::Mat srcp(src_points);
    cv::Mat dstp(dst_points);

    calibration_matrix_ = cv::findHomography(srcp,dstp);
    current_step_ = RECTIFICATION;
    GUI::clearRGBPoints();
    GUI::clearDepthPoints();
}

void Homography::calcHomographyMat(std::vector<cv::Point2f> src_points)
{
    if (src_points.size() != 6)
    {
        ROS_WARN("6 points are needed for homography.");
        return;
    }

    cv::Mat srcp(src_points);
    cv::Mat dstp(dst_points_);

    homography_matrix_ = cv::findHomography(srcp,dstp);
    current_step_ = END;
}

cv::Mat Homography::calibrate(cv::Mat image)
{
    cv::Mat result;
    cv::warpPerspective(image,result,calibration_matrix_,cv::Size(640,480));
    return result;
}

cv::Mat Homography::rectify(cv::Mat image)
{
    cv::Mat result;
    cv::warpPerspective(image,result,homography_matrix_,cv::Size(640,480));
    return result;
}

bool Homography::isHomographyDone()
{
    return current_step_ == END;
}

bool Homography::isCalibrationDone()
{
    return current_step_ > CALIBRATION;
}