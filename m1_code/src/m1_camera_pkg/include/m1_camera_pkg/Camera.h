#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

// Class for cameras and useful functionalities.
class Camera
{
private:
    // Camera
    int cam_idx;
    std::string cam_type, cam_name;

    // ROS
    ros::NodeHandle _nh;
    image_transport::Publisher _pub;
    image_transport::Subscriber _sub;
    sensor_msgs::ImagePtr msg;

public:
    Camera(bool feeds)
    {
        get_params();
        image_transport::ImageTransport img_tr(_nh);
        if (feeds)
        {
            _sub = img_tr.subscribe("camera/feeds", 1, &Camera::show_feeds_cb, this);
        }
        else
        {
            _pub = img_tr.advertise("camera/feeds", 1);
        }
    };

    double get_luminosity();
    void get_params();
    void show_feeds_cb(const sensor_msgs::ImageConstPtr &msg);
    void stream_feeds();
};

#endif