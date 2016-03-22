/**
 * @file   vision.cpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Vision class
 */

#include <unball/vision/vision.hpp>

Vision& Vision::getInstance()
{
    static Vision vision;
    return vision;
}

/**
 * Execute vision processing.
 */
void Vision::run()
{
    // Run calibration-related alterations on original image

    Segmenter::getInstance().runSegmentationAlgorithms();
    // Tracker::getInstance().runTracking();
}
