/**
 * @file   tracked_robot.hpp
 * @author Matheus Vieira Portela
 * @date   31/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot class definition for tracker
 */

#ifndef UNBALL_VISION_TRACKED_ROBOT_H_
#define UNBALL_VISION_TRACKED_ROBOT_H_

#include <ros/ros.h>

#include <unball/vision/tracked_object.hpp>

class TrackedRobot : public TrackedObject
{
  public:
    TrackedRobot();
    ~TrackedRobot();

    void track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame);

  private:

};

#endif // UNBALL_VISION_TRACKED_ROBOT_H_
