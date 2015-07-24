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

    /**
     * Finds the path for the unball package with getPath, then appends the rest of the file path and file name.
     */
    calib_matrix_file_name_ = ros::package::getPath("unball");
    rect_matrix_file_name_ = calib_matrix_file_name_;
    calib_matrix_file_name_.append("/data/calibration_matrix.txt");
    rect_matrix_file_name_.append("/data/rectification_matrix.txt");
}

void Homography::loadConfig()
{
    bool shouldCalibrate, shouldRectify;
    ros::param::get("/vision/homography/calibrate", shouldCalibrate);
    ros::param::get("/vision/homography/rectify", shouldRectify);
    ros::param::get("/vision/homography/overwrite_calib_matrix", overwrite_calibration_matrix_);
    ros::param::get("/vision/homography/overwrite_rect_matrix", overwrite_rectification_matrix_);
    if (shouldCalibrate)
    {
        current_step_ = CALIBRATION;
    }
    else
    {
        current_step_ = (shouldRectify ? RECTIFICATION : END);
        loadMatrix(calib_matrix_file_name_, calibration_matrix_);
        if (current_step_ == END)
            loadMatrix(rect_matrix_file_name_, homography_matrix_);
    }
}

/**
 * Executes the homography algorithm. The homography algorithm is based on steps. Each step must know wich step comes
 * after itself, and is in charge of changing to the next step.
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

    if (overwrite_calibration_matrix_)
        saveMatrix(calib_matrix_file_name_, calibration_matrix_);
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

    if (overwrite_rectification_matrix_)
        saveMatrix(rect_matrix_file_name_, homography_matrix_);
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

/**
 * Saves a matrix to a text file
 * @param file_name the path to the file
 * @param matrix the matrix to be saved
 */
void Homography::saveMatrix(std::string file_name, cv::Mat &matrix)
{
    ROS_DEBUG("Saving matrix to: %s", file_name.c_str());
    std::ofstream file(file_name.c_str());

    if (not file.is_open())
    {
        ROS_ERROR("Error! Could not open file %c%s%c", 34, file_name.c_str(), 34);
        return;
    }

    file << matrix.rows << std::endl << matrix.cols << std::endl;
    for (int i = 0; i < matrix.rows; ++i)
        for (int j = 0; j < matrix.cols; ++j)
            file << matrix.at<double>(j,i) << std::endl;

    file.close();
}

/**
 * Loads a matrix from a text file
 * @param file_name the path to the file
 * @param matrix the matrix to be loaded
 */
void Homography::loadMatrix(std::string file_name, cv::Mat &matrix)
{
    ROS_DEBUG("Loading matrix at: %s", file_name.c_str());
    std::ifstream file(file_name.c_str());

    if (not file.is_open())
    {
        ROS_ERROR("Error! Could not open file %c%s%c", 34, file_name.c_str(), 34);
        return;
    }

    int rows, cols;
    file >> rows;
    file >> cols;
    cv::Mat loaded_matrix(rows, cols, CV_64F);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            file >> loaded_matrix.at<double>(j,i);

    matrix = loaded_matrix;
    file.close();
}
