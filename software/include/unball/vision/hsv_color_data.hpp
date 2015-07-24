/**
 * @file   robot_data.hpp
 * @author Gabriel Naves da Silva
 * @date   24/07/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot data
 */

#ifndef UNBALL_VISION_HSV_COLOR_DATA_H_
#define UNBALL_VISION_HSV_COLOR_DATA_H_

#include <string>

struct HSVColorData
{
    std::string name;
    int min_hue, min_sat, min_val;
    int max_hue, max_sat, max_val;
};

#endif // UNBALL_VISION_HSV_COLOR_DATA_H_
