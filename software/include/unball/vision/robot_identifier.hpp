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

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <unball/vision/robot_data.hpp>
#include <unball/vision/hsv_color_data.hpp>

class RobotIdentifier
{
  public:
    void loadConfig();
    RobotData identifyRobot(cv::Mat rgb_frame, std::vector<cv::Point> contour);

  private:
    void calculateDiagonalPoints(std::vector<cv::Point> contour, cv::Point &farthest_point, cv::Point &opposite_point,
                                 cv::Point center);
    void calculateColorPoints(cv::Mat rgb_frame, cv::Point farthest_point, cv::Point opposite_point, cv::Point center,
                              cv::Vec3b &value_f, cv::Vec3b &value_o);
    void calculateOrientation();

    cv::Point calculateCenterPosition(cv::Rect tracking_window);
    double distanceBetweenPoints(cv::Point a, cv::Point b);
    cv::Point calculateOppositePoint(cv::Point point, cv::Point reference);
    cv::Point calculateMidPoint(cv::Point point, cv::Point reference);
    void setTeamParameters(RobotData &data, RobotData::Team team, cv::Scalar team_color);

    bool isPointColor(std::string color, cv::Vec3b hsv_values);
    bool isPointRed(cv::Vec3b hsv_values);
    bool isPointPink(cv::Vec3b hsv_values);

    int hsv_min_s_, hsv_min_v_;

    std::map<std::string, HSVColorData> color_map_;
};

#endif // UNBALL_VISION_ROBOT_IDENTIFIER_H_
