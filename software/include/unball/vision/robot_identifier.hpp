/**
 * @file   robot_identifier.hpp
 * @author Gabriel Naves da Silva
 * @date   24/07/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Defines the robot identifier class
 * This class is used to identify a robot, using the position of the robot and the rgb image of the field.
 */

#ifndef UNBALL_VISION_ROBOT_IDENTIFIER_H_
#define UNBALL_VISION_ROBOT_IDENTIFIER_H_

#include <map>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>

#include <unball/vision/robot_data.hpp>

class RobotIdentifier
{
  public:
    void loadConfig();
    RobotData identifyRobot(cv::Mat rgb_frame, std::vector<cv::Point> contour, cv::Rect boundingRect);

  private:
    void identifyTeam(RobotData &data, cv::RotatedRect robot, cv::Mat rgb_frame);

    cv::Point2f calculatePointAtMiddle(cv::Point2f a, cv::Point2f b);

    bool isPointBlue(cv::Vec3b hsv_values);
    bool isPointYellow(cv::Vec3b hsv_values);
    bool isPointRed(cv::Vec3b hsv_values);
    bool isPointGreen(cv::Vec3b hsv_values);
    bool isPointPurple(cv::Vec3b hsv_values);

    std::string team_color_;
};

#endif // UNBALL_VISION_ROBOT_IDENTIFIER_H_
