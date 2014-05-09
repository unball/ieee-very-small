/**
 * @file   vision.hpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Vision class
 *
 * Defines computer vision class
 */

#ifndef UNBALL_VISION_H_
#define UNBALL_VISION_H_

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

class Vision
{
  public:
    Vision();
    void findFieldCenter();
    void run();
    void setCameraFrame(cv_bridge::CvImage camera_frame, int image_type);
    float getRobotLocation(int robot_index);
    
    bool has_field_center_;
    
    static const int RGB_IMAGE = 1, DEPTH_IMAGE = 2;
    
  private:
    void segmentDepth();
    void segmentImage();
    void findAngle(int robot_number);
    
    cv_bridge::CvImage rgb_frame_;
    cv_bridge::CvImage depth_frame_;
    float robot_location_[6]; // Final robot location (in centimeters)
    float ball_location_; // Final ball location (in centimeters)
    float robot_angle_[6]; // The angle of each robot (relative to the center of the field)
};

#endif  // UNBALL_VISION_H_
