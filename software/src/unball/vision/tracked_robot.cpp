/**
 * @file   tracked_robot.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   05/08/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot class implementation for tracker
 */

#include <unball/vision/tracked_robot.hpp>

TrackedRobot::TrackedRobot()
{
    type_ = ROBOT;
}

TrackedRobot::~TrackedRobot()
{
}

void TrackedRobot::setMeasurementConversion(MeasurementConversion *mc)
{
    measurement_conversion_ = mc;
}

void TrackedRobot::track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{

}

void TrackedRobot::draw(cv::Mat &frame)
{
    cv::Point2f vertices[4];
    robot_outline_.points(vertices);
    for (int i = 0; i < 4; i++)
        cv::line(frame, vertices[i], vertices[(i+1)%4], robot_color_);
    cv::Point2f tmp_point(position_.x+(cos(orientation_)*20), position_.y+(sin(orientation_)*20));
    cv::line(frame, position_, tmp_point, robot_color_);
}

void TrackedRobot::setPosition(RobotData data)
{
    position_ = data.center_position;
    orientation_ = data.orientation;
    tracking_window_ = data.tracking_window;
    robot_color_ = data.robot_color;
    robot_outline_ = data.robot_outline;
}

std::vector<float> TrackedRobot::getRobotPose()
{
    std::vector<float> pose(3);
    cv::Point2f metric_position = measurement_conversion_->convertToMetric(position_);
    pose[0] = metric_position.x;
    pose[1] = metric_position.y;
    pose[2] = orientation_;

    return pose;
}

cv::Point TrackedRobot::getPixelPosition()
{
    return position_;
}
