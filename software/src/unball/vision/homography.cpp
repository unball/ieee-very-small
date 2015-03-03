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

Homography::Homography(float scale)
{
    scale_ = scale;
    is_done_ = false;

    dst_points_.push_back(scale_*cv::Point2f(0.0,45.0));
    dst_points_.push_back(scale_*cv::Point2f(85.0,0.0));
    dst_points_.push_back(scale_*cv::Point2f(170.0,45.0));

    dst_points_.push_back(scale_*cv::Point2f(170.0,85.0));
    dst_points_.push_back(scale_*cv::Point2f(85.0,130.0));
    dst_points_.push_back(scale_*cv::Point2f(0.0,85.0));
}

void Homography::calcHomographyMat(std::vector<cv::Point2f> src_points)
{
    if (src_points.size() != 6)
    {
        ROS_ERROR("6 points are needed for homography.");
    }

    cv::Mat srcp(src_points);
    cv::Mat dstp(dst_points_);

    homography_matrix_ = cv::findHomography(srcp,dstp);
    is_done_ = true;
}

cv::Mat Homography::transform(cv::Mat image)
{
    cv::Mat result;
    cv::warpPerspective(image,result,homography_matrix_,cv::Size(scale_*170,scale_*130));
    return result;
}

bool Homography::isHomographyDone()
{
    return is_done_;
}