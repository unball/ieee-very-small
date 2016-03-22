/**
 * @file   segmentation_algorithm.hpp
 * @author Gabriel Naves da Silva
 * @date   21/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  SegmentationAlgorithm class
 */

#include <unball/vision/segmentation_algorithm.hpp>

SegmentationAlgorithm::SegmentationAlgorithm()
{
    name_ = "SegmentationAlgorithm_BaseClass";
}

cv::Mat SegmentationAlgorithm::getSegmentationOutput()
{
    return output_image_;
}

bool SegmentationAlgorithm::isName(std::string name)
{
    return name == name_;
}
