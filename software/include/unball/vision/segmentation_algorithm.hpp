/**
 * @file   segmentation_algorithm.hpp
 * @author Gabriel Naves da Silva
 * @date   21/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  SegmentationAlgorithm class
 *
 * Defines the SegmentationAlgorithm class
 *
 * The SegmentationAlgorithm class defines a common interface to all different segmentation
 * algorithms that may be implemented within this project.
 */

#ifndef UNBALL_SEGMENTATION_ALGORITHM_H_
#define UNBALL_SEGMENTATION_ALGORITHM_H_

#include <string>

#include <opencv2/opencv.hpp>

class SegmentationAlgorithm
{
  public:
    SegmentationAlgorithm();
    virtual ~SegmentationAlgorithm() {}

    virtual void run() = 0;

    cv::Mat getSegmentationOutput();
    bool isName(std::string name);

  protected:
    std::string name_;
    cv::Mat output_image_;
};

#endif // UNBALL_SEGMENTATION_ALGORITHM_H_
