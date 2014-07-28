/**
 * @file   preprocessor.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the computer vision preprocessor module
 */

#include <unball/vision/preprocessor.hpp>

#include <iostream> // TODO: Remove this when printMeanMinMax is removed

Preprocessor::Preprocessor()
{
    window_name_ = "Preprocessor";
    cv::namedWindow(window_name_);
}

Preprocessor::~Preprocessor()
{
    cv::destroyWindow(window_name_);
}

/**
 * TODO: Comment this later
 */
cv::Mat Preprocessor::preprocess(cv::Mat image)
{
    cv::Mat preprocessed;

    // Smoother images are better for segmentation
    cv::medianBlur(image, preprocessed, 5);

    // Show results
    cv::imshow(window_name_, preprocessed);
    cv::waitKey(1);

    return preprocessed;
}

/**
 * TODO (gabri.navess@gmail.com): This is a temporary method.
 * It is here so I don't forget how to do this. ;P
 * It will be removed later.
 * 
 * @param image the image to be preprocessed
 */
void Preprocessor::printMeanMinMax(cv::Mat image)
{
    unsigned short int min, max;
    unsigned int mean, counter;
    max = 0; min = 10000; mean = counter = 0;

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            if (image.at<unsigned short int>(i, j) != 0)
            {
                counter++;
                mean += image.at<unsigned short int>(i, j);
                if (image.at<unsigned short int>(i, j) < min) min = image.at<unsigned short int>(i, j);
                if (image.at<unsigned short int>(i, j) > max) max = image.at<unsigned short int>(i, j);
            }
        }
    }

    mean = mean/counter;
    //std::cout << "mean: " << mean << std::endl;
    //std::cout << "Max: " << max << std::endl;
    //std::cout << "Min: " << min << std::endl << std::endl;
}

/**
 * Prototype method for the preprocessing of the depth images. For now, it doesn't do much, just prints
 * the maximum and minimum pixel values, and the mean, on the screen.
 * 
 * @param image the depth image to be preprocessed, it has to be a 16-bit unsigned image with one dimension (16UC1)
 */
void Preprocessor::preprocessDepth(cv::Mat image)
{
    printMeanMinMax(image);
}
