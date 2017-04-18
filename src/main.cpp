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
Mat src;

// Callback functions.
// Get sensor_msgs::Image from Webcam-Topic and convert to openCV format.
void getImage(const sensor_msgs::Image::ConstPtr& incoming_image)
{
      cout<<"Got image."<<endl;
      // Create a pointer, where the incoming ros-image gets assigned to.
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);

      // Check for valid image.
      if (!(cv_ptr->image).data)
      {
        std::cout<<"Failed to load image"<<std::endl;

      }
      // Assign the incoming image to src (source).
      src = cv_ptr->image;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "arc_lane_following_autopilot");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("maincamera/image_raw", 10, getImage);
  ros::spin();
  return 0;
}

// Source
// http://www.transistor.io/revisiting-lane-detection-using-opencv.html
