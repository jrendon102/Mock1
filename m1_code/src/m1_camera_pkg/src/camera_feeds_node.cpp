/* Subscriber node to view live video feeds from usb camera. */

#include <m1_camera_pkg/Camera.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_feeds_node");
    Camera cam_feeds = Camera(true);
    ros::spin();
    return 0;
}
