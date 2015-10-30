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
    void calibrateColors();
    static void mouseCallback(int event, int x, int y, int, void*);

  private:
    void identifyTeam(RobotData &data, cv::RotatedRect robot, cv::Mat rgb_frame);
    void loadColors();
    void createTrackbars();
    cv::Point2f calculatePointAtMiddle(cv::Point2f a, cv::Point2f b);
    bool isPointBlue(cv::Vec3b hsv_values);
    bool isPointYellow(cv::Vec3b hsv_values);
    bool isPointRed(cv::Vec3b hsv_values);
    bool isPointGreen(cv::Vec3b hsv_values);
    bool isPointPurple(cv::Vec3b hsv_values);
    int blue_min_[3], blue_max_[3];
    int yellow_min_[3], yellow_max_[3];
    int green_min_[3], green_max_[3];
    int red_min_[3], red_max_[3];
    int purple_min_[3], purple_max_[3];
    int hsv_min_s_, hsv_min_v_;
    std::string window_name_;
    std::string team_color_;
    static cv::Point2f point_;

};

#endif // UNBALL_VISION_ROBOT_IDENTIFIER_H_
