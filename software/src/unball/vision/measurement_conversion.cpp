/**
 * @file   measurement_conversion.hpp
 * @author Gabriel Naves da Silva
 * @date   29/06/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Measurement conversion system implementation file for vision module
 */

#include <unball/vision/measurement_conversion.hpp>

MeasurementConversion::MeasurementConversion()
{
    has_calculated_parameters_ = false;
    field_metric_width_ = field_metric_height_ = 0;
}

void MeasurementConversion::loadConfig()
{
    ros::param::get("/vision/measurement_conversion/field_width", field_metric_width_);
    ros::param::get("/vision/measurement_conversion/field_height", field_metric_height_);
}

/**
 * Calculates the necessary parameters for unit conversion (from pixel to metric, and vice-versa)
 * @param field_pixel_width The value for the width of the field, given in pixels
 * @param field_pixel_height The value for the height of the field, given in pixels
 */
void MeasurementConversion::calculateConversion(float field_pixel_width, float field_pixel_height)
{
    ROS_DEBUG("Calculating the conversion parameters");

    if (field_pixel_width < 0 or field_pixel_height < 0)
        ROS_ERROR("Invalid field pixel measurements given");

    conversion_parameters_.first = field_metric_width_ / field_pixel_width;
    conversion_parameters_.second = field_metric_height_ / field_pixel_height;

    ROS_DEBUG("Finished calculating measurement unit conversion parameters.");
    ROS_DEBUG("Conversion parameter for the x axis: %f", conversion_parameters_.first);
    ROS_DEBUG("Conversion parameter for the y axis: %f", conversion_parameters_.second);

    has_calculated_parameters_ = true;
}

/**
 * Converts a given pixel coordinate into the equivalent metric coordinate
 * @param point_in_pixel The point to convert
 * @return The resulting point
 */
cv::Point MeasurementConversion::convertToMetric(cv::Point point_in_pixel)
{
    if (not has_calculated_parameters_)
    {
        ROS_ERROR("Trying to make a conversion without calculating the parameters first");
        return cv::Point();
    }
    return cv::Point(point_in_pixel.x * conversion_parameters_.first,
                     point_in_pixel.y * conversion_parameters_.second);
}

/**
 * Converts a given metric coordinate into the equivalent pixel coordinate
 * @param point_in_metric The point to convert
 * @return The resulting point
 */
cv::Point MeasurementConversion::convertToPixel(cv::Point point_in_metric)
{
    if (not has_calculated_parameters_)
    {
        ROS_ERROR("Trying to make a conversion without calculating the parameters first");
        return cv::Point();
    }
    return cv::Point(point_in_metric.x * (1 / conversion_parameters_.first),
                     point_in_metric.y * (1 / conversion_parameters_.second));
}
