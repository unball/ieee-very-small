/**
 * @file   homography.hpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the computer vision homography module
 */

#include <unball/vision/homography.hpp>

Homography::Homography()
{
    is_done_ = false;

    dst_points_.push_back(cv::Point2f(0.0,0.0));
    dst_points_.push_back(cv::Point2f(320.0,0.0));
    dst_points_.push_back(cv::Point2f(640.0,0.0));

    dst_points_.push_back(cv::Point2f(0.0,480.0));
    dst_points_.push_back(cv::Point2f(320.0,480.0));
    dst_points_.push_back(cv::Point2f(640.0,480.0));
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
    is_done_ = true;
}

cv::Mat Homography::transform(cv::Mat image)
{
    cv::Mat result;
    cv::warpPerspective(image,result,homography_matrix_,cv::Size(640,480));
    return result;
}

bool Homography::isHomographyDone()
{
    return is_done_;
}
