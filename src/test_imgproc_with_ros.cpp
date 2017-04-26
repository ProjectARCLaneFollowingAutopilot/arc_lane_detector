#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

// Global variables.
// Variable to save the incoming image from the webcam globally (maybe not needed).
Mat src_img;

// Callback functions.
// Function sets parameters if first image arrives. Otherwise IPM is done.
void imageCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
  // Create a pointer, where the incoming ros-image gets assigned to.
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);

  // Check for valid image.
  if (!(cv_ptr->image).data)
  {
    std::cout<<"Failed to load image"<<std::endl;

  }

  // FLIP THE TABLE. Flip and save the image to global variable.
  cv::flip(cv_ptr->image, src_img, 0);

}


int main(int argc, char* argv[])
{
  // Initialize the node.
  ros::init(argc, argv, "test_imgproc_with_ros");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::spin();

  return 0;
}

/* This program will be used to test different imamgeprocessing techniques in.
For example: Filter, color channels, edge detections, gradient operators,...?
*/
