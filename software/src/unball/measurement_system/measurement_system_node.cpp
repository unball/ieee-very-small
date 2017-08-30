#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <vision/PixelToMetricConversionMessage.h>
#include <unball/MeasurementSystemMessage.h>

void receivePxToMMessage(const vision::PixelToMetricConversionMessage::ConstPtr &msg_s);
void filter();

unball::MeasurementSystemMessage message;

ros::Publisher publisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "measurement_system_node");

    ros::NodeHandle n;
    ros::Rate loop_rate(10); // Hz

    ros::Subscriber sub = n.subscribe("pixel_to_metric_conversion_topic", 1, receivePxToMMessage);
    publisher = n.advertise<unball::MeasurementSystemMessage>("measurement_system_topic", 1);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void receivePxToMMessage(const vision::PixelToMetricConversionMessage::ConstPtr &msg_v)
{
    std::vector<float> x(6), y(6), th(6);
    std::vector<float> ball_location(2);

    ROS_INFO("\n\n[MeasurementNode]:receivePxToMMessage");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, msg_v->x[robot_index], msg_v->y[robot_index],
            msg_v->th[robot_index]);

        message.x[robot_index] = msg_v->x[robot_index];
        message.y[robot_index] = msg_v->y[robot_index];
        message.th[robot_index] = msg_v->th[robot_index];
    }
    message.ball_x = msg_v->ball_x;
    message.ball_y = msg_v->ball_y;

    ROS_INFO("\n\n[MeasurementNode]: Sending measurement system message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, message.x[robot_index], message.y[robot_index],
            message.th[robot_index]);
    }
    ROS_INFO("Ball: x: %f, y: %f", message.ball_x, message.ball_y);

    publisher.publish(message);
}
