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
#include <unball/vision/hsv_color_data.hpp>

class RobotIdentifier
{
  public:
    void loadConfig();
    RobotData identifyRobot(cv::Mat rgb_frame, std::vector<cv::Point> contour, cv::Rect boundingRect);

  private:
    void loadShirtImages();
    cv::Mat calculateHistogram(cv::Mat img);
    void identifyTeam(RobotData &data, cv::RotatedRect robot, cv::Mat rgb_frame);

    cv::Point2f calculatePointAtMiddle(cv::Point2f a, cv::Point2f b);

    bool isPointColor(std::string color, cv::Vec3b hsv_values);

    bool isPointBlue(cv::Vec3b hsv_values);
    bool isPointYellow(cv::Vec3b hsv_values);
    bool isPointRed(cv::Vec3b hsv_values);
    bool isPointGreen(cv::Vec3b hsv_values);
    bool isPointPurple(cv::Vec3b hsv_values);

    int hsv_min_s_, hsv_min_v_;

    std::string team_color_;
    cv::Mat shirt_images_[3];
    cv::Mat shirt_histograms_[3];

    std::map<std::string, HSVColorData> color_map_;
};

#endif // UNBALL_VISION_ROBOT_IDENTIFIER_H_
