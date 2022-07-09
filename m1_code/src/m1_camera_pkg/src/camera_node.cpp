/* Publisher node to stream/publish CvImage as ros msg. */

#include <m1_camera_pkg/Camera.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_node");
    Camera cam_feeds = Camera(false);
    return 0;
}
