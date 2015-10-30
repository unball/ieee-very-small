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

cv::Point2f RobotIdentifier::point_ = cv::Point2f(0,0);


void RobotIdentifier::loadConfig()
{
    ros::param::get("/vision/segmenter/hsv_min_s", hsv_min_s_);
    ros::param::get("/vision/segmenter/hsv_min_v", hsv_min_v_);
    ros::param::get("/vision/tracker/team", team_color_);
    loadColors();
    createTrackbars();
    window_name_ = "Color Trackbar";
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

void RobotIdentifier::loadColors()
{
    ros::param::get("/vision/color/blue_min_h", blue_min_[0]);
    ros::param::get("/vision/color/blue_max_h", blue_max_[0]);
    ros::param::get("/vision/color/blue_min_s", blue_min_[1]);
    ros::param::get("/vision/color/blue_max_s", blue_max_[1]);
    ros::param::get("/vision/color/blue_min_v", blue_min_[2]);
    ros::param::get("/vision/color/blue_max_v", blue_max_[2]);
    ros::param::get("/vision/color/yellow_min_h", yellow_min_[0]);
    ros::param::get("/vision/color/yellow_max_h", yellow_max_[0]);
    ros::param::get("/vision/color/yellow_min_s", yellow_min_[1]);
    ros::param::get("/vision/color/yellow_max_s", yellow_max_[1]);
    ros::param::get("/vision/color/yellow_min_v", yellow_min_[2]);
    ros::param::get("/vision/color/yellow_max_v", yellow_max_[2]);
    ros::param::get("/vision/color/red_min_h", red_min_[0]);
    ros::param::get("/vision/color/red_max_h", red_max_[0]);
    ros::param::get("/vision/color/red_min_s", red_min_[1]);
    ros::param::get("/vision/color/red_max_s", red_max_[1]);
    ros::param::get("/vision/color/red_min_v", red_min_[2]);
    ros::param::get("/vision/color/red_max_v", red_max_[2]);
    ros::param::get("/vision/color/green_min_h", green_min_[0]);
    ros::param::get("/vision/color/green_max_h", green_max_[0]);
    ros::param::get("/vision/color/green_min_s", green_min_[1]);
    ros::param::get("/vision/color/green_max_s", green_max_[1]);
    ros::param::get("/vision/color/green_min_v", green_min_[2]);
    ros::param::get("/vision/color/green_max_v", green_max_[2]);
    ros::param::get("/vision/color/purple_min_h", purple_min_[0]);
    ros::param::get("/vision/color/purple_max_h", purple_max_[0]);
    ros::param::get("/vision/color/purple_min_s", purple_min_[1]);
    ros::param::get("/vision/color/purple_max_s", purple_max_[1]);
    ros::param::get("/vision/color/purple_min_v", purple_min_[2]);
    ros::param::get("/vision/color/purple_max_v", purple_max_[2]);
}

void RobotIdentifier::createTrackbars()
{

    cv::namedWindow(window_name_);
    cv::createTrackbar("BLUEMINH", window_name_, &blue_min_[0], 255);
    cv::createTrackbar("BLUEMAXH", window_name_, &blue_max_[0], 256);
    cv::createTrackbar("BLUEMINV", window_name_, &blue_min_[1], 256);
    cv::createTrackbar("BLUEMAXV", window_name_, &blue_max_[1], 256);
    cv::createTrackbar("BLUEMAXS", window_name_, &blue_min_[2], 256);
    cv::createTrackbar("BLUEMINS", window_name_, &blue_max_[2], 256);
    cv::setMouseCallback(window_name_, mouseCallback);
    cv::destroyWindow(window_name_);
    cv::namedWindow(window_name_);
    // cv::createTrackbar("YELLOWMINH", window_name_, &yellow_min_[0], 256);
    // cv::createTrackbar("YELLOWMAXH", window_name_, &yellow_max_[0], 256);
    // cv::createTrackbar("YELLOWMINV", window_name_, &yellow_min_[1], 256);
    // cv::createTrackbar("YELLOWMAXV", window_name_, &yellow_max_[1], 256);
    // cv::createTrackbar("YELLOWMAXS", window_name_, &yellow_min_[2], 256);
    // cv::createTrackbar("YELLOWMINS", window_name_, &yellow_max_[2], 256);
    
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

void RobotIdentifier::mouseCallback(int event, int x, int y, int, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN)
        point_ = cv::Point2f(x, y);
    
   
}