#include <cv_bridge/cv_bridge.h>
#include <m1_camera_pkg/Camera.h>

// Get the luminosity value of the image.
double Camera::get_luminosity()
{
    return 10;
}

// Gets ROS parameters set by camera config file.
void Camera::get_params()
{
    _nh.getParam("/camera_config/camera/index", cam_idx);
    _nh.getParam("/camera_config/camera/name", cam_name);
    _nh.getParam("/camera_config/camera/type", cam_type);
}

//  Callback function to view the video feeds.
void Camera::show_feeds_cb(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::imshow(cam_name, cv_bridge::toCvShare(msg, "bgr8")->image);
        char c = (char)cv::waitKey(25);
        if (c == 27)
        {
            ros::shutdown();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from [%s] to bgr8.", msg->encoding.c_str());
    }
}

// Convert the video feeds as a ros msg. This will be used so that video feeds can be streamed remotely.
void Camera::stream_feeds()
{
    ROS_INFO("Setting up video streaming for [%s].", cam_name.c_str());

    cv::namedWindow(cam_name);
    cv::VideoCapture cap(cam_idx, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        ROS_ERROR("Could not open [%s]", cam_name.c_str());
        ros::shutdown();
    }

    ros::Rate loop(25);
    while (true)
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
        {
            ROS_ERROR("Frame is empty!");
            break;
        }

        // Convert CvImage to ROS msg and publish msg.
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        _pub.publish(msg);
        loop.sleep();
    }
    cap.release();
}
