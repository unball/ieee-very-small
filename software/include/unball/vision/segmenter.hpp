/**
 * @file   segmenter.hpp
 * @author Gabriel Naves da Silva
 * @date   21/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Segmenter class
 *
 * Defines the Segmenter class
 *
 * The segmenter class manages the available segmentation algorithms.
 */

#ifndef UNBALL_SEGMENTER_H_
#define UNBALL_SEGMENTER_H_

#include <vector>

class Segmenter
{
  public:

    void runSegmentationAlgorithms();

    static Segmenter& getInstance();


  private:
};

#endif // UNBALL_SEGMENTER_H_
