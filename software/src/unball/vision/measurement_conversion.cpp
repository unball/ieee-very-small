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
    ROS_ERROR("Calculating the conversion parameters");

    if (field_pixel_width < 0 or field_pixel_height < 0)
        ROS_ERROR("Invalid field pixel measurements given");

    conversion_parameters_.x = field_metric_width_ / field_pixel_width;
    conversion_parameters_.y = field_metric_height_ / field_pixel_height;

    ROS_DEBUG("Field dimensions: (%f,%f)", field_pixel_width, field_pixel_height);
    ROS_DEBUG("Conversion parameter for the x axis: %f", conversion_parameters_.x);
    ROS_DEBUG("Conversion parameter for the y axis: %f", conversion_parameters_.y);

    has_calculated_parameters_ = true;
}

/**
 * Sets the field center, for conversion from a point to the magnitude and angle notation used by the
 * robot soccer AI system
 * @param field_center the center of the field, in pixels
 */
void MeasurementConversion::setFieldCenter(cv::Point field_center)
{
    ROS_DEBUG("Setting the field center on the measurement conversion system");
    field_center_ = field_center;
}

/**
 * Converts a given pixel coordinate into the magnitude and angle notation used by the robot soccer AI system.
 * @param point_in_pixel The point to convert
 * @return The resulting magnitude and angle relative to the center of the field
 */
cv::Point2f MeasurementConversion::pixelToMagnitudeAndAngle(cv::Point point_in_pixel)
{
    if (not has_calculated_parameters_)
    {
        ROS_ERROR("Trying to make a conversion without calculating the parameters first");
        return cv::Point2f();
    }
    float magnitude, angle;
    magnitude = sqrt(pow(point_in_pixel.x - field_center_.x, 2) + pow(point_in_pixel.y - field_center_.y, 2));
    // TODO(gabri.navess@gmail.com): Find out whether this angle is correct.
    angle = atan2(point_in_pixel.y-field_center_.y, point_in_pixel.x-field_center_.x);
    return cv::Point2f(magnitude, angle);
}

/**
 * Converts a given pixel coordinate into the equivalent metric coordinate
 * @param point_in_pixel The point to convert
 * @return The resulting point
 */
cv::Point2f MeasurementConversion::convertToMetric(cv::Point point_in_pixel)
{
    if (not has_calculated_parameters_)
    {
        ROS_ERROR("Trying to make a conversion without calculating the parameters first");
        return cv::Point2f();
    }
    return cv::Point2f(point_in_pixel.x * conversion_parameters_.x,
                     point_in_pixel.y * conversion_parameters_.y);
}

/**
 * Converts a given metric coordinate into the equivalent pixel coordinate
 * @param point_in_metric The point to convert
 * @return The resulting point
 */
cv::Point MeasurementConversion::convertToPixel(cv::Point2f point_in_metric)
{
    if (not has_calculated_parameters_)
    {
        ROS_ERROR("Trying to make a conversion without calculating the parameters first");
        return cv::Point();
    }
    return cv::Point(point_in_metric.x * (1 / conversion_parameters_.x),
                     point_in_metric.y * (1 / conversion_parameters_.y));
}
