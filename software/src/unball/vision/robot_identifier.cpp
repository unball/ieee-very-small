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
    ros::param::get("/vision/tracker/team", team_color_);
}

RobotData RobotIdentifier::identifyRobot(cv::Mat rgb_frame, std::vector<cv::Point> contour, cv::Rect boundingRect)
{
    RobotData data;

    data.robot_outline = cv::minAreaRect(contour);
    data.center_position = cv::Point(data.robot_outline.center.x, data.robot_outline.center.y);
    data.tracking_window = cv::boundingRect(contour);

    identifyTeam(data, data.robot_outline, rgb_frame);

    return data;
}

void RobotIdentifier::identifyTeam(RobotData &data, cv::RotatedRect robot, cv::Mat rgb_frame)
{
    cv::Mat hsv;
    cv::Point2f test_points[4];
    cv::Point2f vertices[4];
    robot.points(vertices);
    cv::cvtColor(rgb_frame, hsv, CV_BGR2HSV);
    for (int i = 0; i < 4; ++i)
    {
        test_points[i] = calculatePointAtMiddle(robot.center, vertices[i]);
        cv::Vec3b hsv_value = hsv.at<cv::Vec3b>(test_points[i].y, test_points[i].x);
        if ((team_color_ == "Blue" and isPointBlue(hsv_value)) or
            (team_color_ == "Yellow" and isPointYellow(hsv_value)))
        {
            data.team = RobotData::ALLY;
            data.orientation = (robot.angle+(90*((i+1)%4)))*2*M_PI/360.0;
            cv::Point2f id_test_point = calculatePointAtMiddle(robot.center, vertices[(i+2)%4]);
            cv::Vec3b hsv_id_point = hsv.at<cv::Vec3b>(id_test_point.y, id_test_point.x);
            if (isPointRed(hsv_id_point))
            {
                data.id = 0;
                data.robot_color = cv::Scalar(0, 0, 255);
            }
            else if (isPointGreen(hsv_id_point))
            {
                data.id = 1;
                data.robot_color = cv::Scalar(0, 255, 0);
            }
            else if (isPointPurple(hsv_id_point))
            {
                data.id = 2;
                data.robot_color = cv::Scalar(255, 0, 0);
            }
            else
            {
                ROS_ERROR("Could not identify this robot");
            }
            return;
        }
    }
    data.team = RobotData::OPPONENT;
    data.id = 0;
    data.orientation = robot.angle * 2 * M_PI / 360;
}

cv::Point2f RobotIdentifier::calculatePointAtMiddle(cv::Point2f a, cv::Point2f b)
{
    return cv::Point2f((a.x+b.x)/2,(a.y+b.y)/2);
}

bool RobotIdentifier::isPointBlue(cv::Vec3b hsv_values)
{
    return (hsv_values[0] > 100 and hsv_values[0] <= 150 and hsv_values[1] > 133 and hsv_values[2] > 90);
}

bool RobotIdentifier::isPointYellow(cv::Vec3b hsv_values)
{
    return (hsv_values[0] <= 40 and hsv_values[1] > 133 and hsv_values[2] > 90);
}

bool RobotIdentifier::isPointRed(cv::Vec3b hsv_values)
{
    return (hsv_values[0] > 150 and hsv_values[0] <= 200 and hsv_values[1] > 133 and hsv_values[2] > 90);
}

bool RobotIdentifier::isPointGreen(cv::Vec3b hsv_values)
{
    return (hsv_values[0] > 80 and hsv_values[0] <= 100 and hsv_values[2] > 90);
}

bool RobotIdentifier::isPointPurple(cv::Vec3b hsv_values)
{
    return (hsv_values[0] > 120 and hsv_values[0] <= 140 and hsv_values[2] > 90);
}
