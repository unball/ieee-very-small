/**
 * @file   homography.hpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the computer vision homography class. 
 */

#include <unball/vision/homography.hpp>

/**
 * Homography class constructor. It prepares the dst_points_ vector.
 */
Homography::Homography()
{
    /**
     * The values for the dst_points_ vector are fixed, and were calculated considering the dimensions of the RGB and
     * depth images given by the kinect.
     */
    dst_points_.push_back(cv::Point2f(0.0,0.0));
    dst_points_.push_back(cv::Point2f(320.0,0.0));
    dst_points_.push_back(cv::Point2f(640.0,0.0));

    dst_points_.push_back(cv::Point2f(0.0,480.0));
    dst_points_.push_back(cv::Point2f(320.0,480.0));
    dst_points_.push_back(cv::Point2f(640.0,480.0));
}

void Homography::loadConfig()
{
    bool shouldCalibrate;
    ros::param::get("/vision/homography/calibrate", shouldCalibrate);
    current_step_ = (shouldCalibrate ? CALIBRATION : RECTIFICATION);
    
    if (current_step_ == RECTIFICATION)
        loadCalibrationMatrix();
}

/**
 * Executes the homography algorithm. 
 * @param rgb_points The selected points on RGB image
 * @param depth_points The selected points on depth image
 */
void Homography::run(std::vector<cv::Point2f> rgb_points, std::vector<cv::Point2f> depth_points)
{
    switch (current_step_)
    {
        case CALIBRATION:
            calcCalibrationMat(depth_points, rgb_points); // Depth points are matched with rgb points
            break;
        case RECTIFICATION:
            calcHomographyMat(rgb_points); // The points used for rectification are the rgb points
            break;
        default:
            break;
    }
}

/**
 * Calculates the calibration matrix. This matrix will be used so as to match depth images with RGB images.
 * After calculation is complete, proceeds to the next step (rectification) and resets the RGB and depth points.
 * @param src_points the points of the source image
 * @param dst_points the points of the destination image
 */
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

    saveCalibrationMatrix();
}

/**
 * Calculates the rectification matrix. This matrix will be used so as to rectify depth and RGB images.
 * After calculation is complete, proceeds to the next step (end) and resets the RGB and depth points.
 * @param src_points the points of the source image
 */
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
    GUI::clearRGBPoints();
    GUI::clearDepthPoints();
}

void Homography::saveCalibrationMatrix()
{
    // TODO (gabri.navess@gmail.com): Actually save the matrix
}

void Homography::loadCalibrationMatrix()
{
    // TODO (gabri.navess@gmail.com): Actually load the matrix
}

/**
 * Calibrates given images using the calibration matrix and returns the resulting image.
 * @param image the image to be calibrated
 * @return the resulting warped image.
 */
cv::Mat Homography::calibrate(cv::Mat image)
{
    cv::Mat result;
    cv::warpPerspective(image,result,calibration_matrix_,cv::Size(640,480));
    return result;
}

/**
 * Rectifies given images using the rectification matrix and returns the resulting image.
 * @param image the image to be rectified
 * @return the resulting warped image.
 */
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
