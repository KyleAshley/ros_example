#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // imshow is the standard opencv image viewing function
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    // always wait for a breif period after displaying an image to allow for refresh of the window
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  // create a window named "view"
  cv::namedWindow("view");
  cv::startWindowThread();
  
  // subscribe to the image topic 
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
}