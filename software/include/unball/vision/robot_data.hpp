/**
 * @file   robot_data.hpp
 * @author Gabriel Naves da Silva
 * @date   24/07/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot data
 */

#ifndef UNBALL_VISION_ROBOT_DATA_H_
#define UNBALL_VISION_ROBOT_DATA_H_

struct RobotData
{
    int team;
    int id; // defined by config
    cv::Rect tracking_window;
    cv::RotatedRect robot_outline;
    cv::Point center_position; // in pixels
    float orientation;
    cv::Scalar robot_color;

    enum Team
    {
        ALLY,
        OPPONENT
    };
};

#endif // UNBALL_VISION_ROBOT_DATA_H_
