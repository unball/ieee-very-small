/**
 * @file   vision_polygon.hpp
 * @author Gabriel Naves da Silva
 * @date   29/10/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the computer vision polygon module
 */

#include <unball/vision/vision_polygon.hpp>

void VisionPolygon::setPoints(const std::vector<cv::Point2f> &rgb_points)
{
    points_.clear();
    for (int i = 0; i < rgb_points.size(); ++i)
        points_.push_back(cv::Point2f(rgb_points[i].y, rgb_points[i].x));
}

bool VisionPolygon::isPointInside(cv::Point2f point)
{
    int x = point.x, y = point.y;
    int i, j = ((int)points_.size())-1;
    bool oddNodes = false;

    for (i = 0; i < points_.size(); i++)
    {
        if (points_[i].y < y and points_[j].y >= y or points_[j].y < y and points_[i].y >= y)
        {
            if (points_[i].x + (y - points_[i].y) / (points_[j].y - points_[i].y) * (points_[j].x - points_[i].x) < x)
            {
                oddNodes=!oddNodes;
            }
        }
        j = i;
    }

    return oddNodes;
    // for (int i = 0; i < points_.size(); ++i)
    // {
    //     if (i == points_.size()-1)
    //     {
    //         if (not isPointToTheRightOfSegment(points_[i], points_[0], point))
    //             return false;
    //     }
    //     else
    //     {
    //         if (not isPointToTheRightOfSegment(points_[i], points_[i+1], point))
    //             return false;
    //     }
    // }
    // return true;
}

// bool VisionPolygon::isPointToTheRightOfSegment(cv::Point2f seg_start, cv::Point2f seg_end, cv::Point2f point)
// {
//     if (seg_start.x == seg_end.x)


//     // (y - y0) = a(x - x0)
//     // - a = (y - y0)/(x - x0)
//     float a = (seg_end.y - seg_start.y)/(seg_end.x - seg_start.x);
//     // (y = ax + b)
//     // - b = y - ax
//     float b = seg_end.y - a * seg_end.x;
//     // - x = (y - b)/a
//     float x_pos = (point.y - b) / a;
//     if (a == 0)
// }
