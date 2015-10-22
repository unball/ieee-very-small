/**
 * @file   tracked_robot.hpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   31/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot class definition for tracker
 */

#ifndef UNBALL_VISION_TRACKED_ROBOT_H_
#define UNBALL_VISION_TRACKED_ROBOT_H_

#include <vector>

#include <ros/ros.h>

#include <unball/vision/tracked_object.hpp>
#include <unball/vision/robot_data.hpp>
#include <unball/vision/measurement_conversion.hpp>

class TrackedRobot : public TrackedObject
{
  public:
    TrackedRobot();
    ~TrackedRobot();

    void setMeasurementConversion(MeasurementConversion *mc);

    void track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame);
    void draw(cv::Mat &frame);
    void setPosition(RobotData data);

    std::vector<float> getRobotPose();

  private:
    MeasurementConversion *measurement_conversion_;

    cv::Scalar robot_color_;
    cv::RotatedRect robot_outline_;
};

#endif // UNBALL_VISION_TRACKED_ROBOT_H_
