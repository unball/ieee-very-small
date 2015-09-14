/**
 * @file   robot_identifier.cpp
 * @author Gabriel Naves da Silva
 * @date   24/07/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the RobotIdentifier class
 */

#include <unball/vision/robot_identifier.hpp>

void RobotIdentifier::loadConfig()
{
    ros::param::get("/vision/segmenter/hsv_min_s", hsv_min_s_);
    ros::param::get("/vision/segmenter/hsv_min_v", hsv_min_v_);
}

RobotData RobotIdentifier::identifyRobot(cv::Mat rgb_frame, std::vector<cv::Point> contour)
{
    RobotData data;

    data.tracking_window = cv::boundingRect(contour);
    data.center_position = calculateCenterPosition(data.tracking_window);
    data.orientation = calculateOrientation(contour);

    cv::Point farthest_point, opposite_point;
    calculateDiagonalPoints(contour, farthest_point, opposite_point, data.center_position);

    cv::Vec3b value_f, value_o;
    calculateColorPoints(rgb_frame, farthest_point, opposite_point, data.center_position, value_f, value_o);

    if (isPointPink(value_f) or isPointPink(value_o))
        setTeamParameters(data, RobotData::ALLY, cv::Scalar(180, 0, 180));
    else if (isPointRed(value_f) or isPointRed(value_o))
        setTeamParameters(data, RobotData::OPPONENT, cv::Scalar(0, 0, 180));
    else
        ROS_ERROR("[RobotIdentifier]identifyRobot: could not identify the team");

    data.id = 0;

    return data;
}

void RobotIdentifier::calculateDiagonalPoints(std::vector<cv::Point> contour, cv::Point &farthest_point,
                                              cv::Point &opposite_point, cv::Point center)
{
    farthest_point = contour[0];
    float distance = distanceBetweenPoints(center, farthest_point);
    for (int k = 1; k < contour.size(); ++k)
    {
        float current_distance = distanceBetweenPoints(center, contour[k]);
        if (current_distance > distance)
        {
            farthest_point = contour[k];
            distance = current_distance;
        }
    }
    opposite_point = calculateOppositePoint(farthest_point, center);
}

void RobotIdentifier::calculateColorPoints(cv::Mat rgb_frame, cv::Point farthest_point, cv::Point opposite_point,
                                           cv::Point center, cv::Vec3b &value_f, cv::Vec3b &value_o)
{
    cv::Point middle_f = calculateMidPoint(farthest_point, center);
    cv::Point middle_o = calculateMidPoint(opposite_point, center);

    cv::Mat hsv;
    cv::cvtColor(rgb_frame, hsv, CV_BGR2HSV);
    value_f = hsv.at<cv::Vec3b>(middle_f.y,middle_f.x);
    value_o = hsv.at<cv::Vec3b>(middle_o.y,middle_o.x);
}

float RobotIdentifier::calculateOrientation(std::vector<cv::Point> contour)
{
    // Orientation is calculated using moments, according to this:
    // https://en.wikipedia.org/wiki/Image_moment#Examples_2
    cv::Moments moments = cv::moments(contour);
    float theta = atan(2*moments.m11/(moments.m20 - moments.m02))/2;
}

cv::Point RobotIdentifier::calculateCenterPosition(cv::Rect tracking_window)
{
    cv::Point center;
    cv::Point top_left = tracking_window.tl();
    cv::Point bottom_right = tracking_window.br();
    center.x = (top_left.x + bottom_right.x)/2;
    center.y = (top_left.y + bottom_right.y)/2;
    return center;
}

double RobotIdentifier::distanceBetweenPoints(cv::Point a, cv::Point b)
{
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2));
}

cv::Point RobotIdentifier::calculateOppositePoint(cv::Point point, cv::Point reference)
{
    return cv::Point(2 * reference.x - point.x, 2 * reference.y - point.y);
}

cv::Point RobotIdentifier::calculateMidPoint(cv::Point point, cv::Point reference)
{
    return cv::Point(reference.x + (point.x-reference.x)/2, reference.y + (point.y-reference.y)/2);
}

/**
 * Determines whether the given point's color is the one given or not
 * @param color the color being identified
 * @param hsv_values the hsv values of the point
 * @return whether the point is of the given color or not
 */
bool RobotIdentifier::isPointColor(std::string color, cv::Vec3b hsv_values)
{
    if (color_map_.find(color) == color_map_.end())
    {
        ROS_ERROR("[RobotIdentifier]isPointColor: color %s does not exist in color map", color.c_str());
        return false;
    }
    else
    {
        HSVColorData data = color_map_[color];
        return (hsv_values[0] >= data.min_hue and hsv_values[0] <= data.max_hue and
                hsv_values[1] >= data.min_sat and hsv_values[1] <= data.max_sat and
                hsv_values[2] >= data.min_val and hsv_values[2] <= data.max_val);
    }
}

bool RobotIdentifier::isPointRed(cv::Vec3b hsv_values)
{
    if (hsv_values[2] < hsv_min_v_ or hsv_values[1] < hsv_min_s_ or hsv_values[0] < 170)
        return false;
    return true;
}

bool RobotIdentifier::isPointPink(cv::Vec3b hsv_values)
{
    if (hsv_values[2] < hsv_min_v_ or hsv_values[1] < hsv_min_s_ or hsv_values[0] > 175 or hsv_values[1] > 200)
        return false;
    return true;
}

void RobotIdentifier::setTeamParameters(RobotData &data, RobotData::Team team, cv::Scalar team_color)
{
    data.team = team;
    data.team_color = team_color;
}
