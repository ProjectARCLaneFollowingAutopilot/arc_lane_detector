#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;

// Global variables.
IPM test_object;
// Variable to save the incoming image from the webcam globally (maybe not needed).
Mat src_img;
Mat after_ipm;
Mat bird_gray;
Mat bird_canny;
float camera_height = 1.3;      //0.7;         // 1.3 0.78;
float pitch_angle = 90;                 //80.0;
float focal_length_px = 628.0;
// Variable to only set the parameters once.
int counter = 0;

// Callback functions.
// Function sets parameters if first image arrives. Otherwise IPM is done.
void ipmCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
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

  // Give test_object the image.
  test_object.IPM::getImage(src_img);

  // Set the extrinsics and intrinsics of test_object (also saves the transformation matrix of test_object).
  // Do this only the first time an image has been received.
  if(counter == 0)
  {
    cout<<"Got image to set the parameters."<<endl;
    test_object.IPM::setParam(camera_height, pitch_angle, focal_length_px, src_img.cols, src_img.rows);
    counter = counter + 1;
  }

  // Do the IPM transformation on the received image.
  after_ipm = test_object.IPM::invPerspectiveMapping();
  //cv::imshow("IPMed", after_ipm);

  // Convert to grayscale.
  cv::cvtColor(after_ipm, bird_gray, CV_BGR2GRAY );

  // Do filtering.
  cv::medianBlur(bird_gray, bird_gray, 5);

  // Run canny.
  cv::Canny(bird_gray, bird_canny, 40, 130);

  cv::imshow("Canny Bird", bird_canny);

  cv::waitKey(3);
}


int main(int argc, char* argv[])
{
  // Initialize the node.
  ros::init(argc, argv, "test_ipm_with_ros_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, ipmCallback);
  ros::spin();

  // Destroy the object.
  test_object.IPM::~IPM();
  return 0;
}

/*
This program will be used to test the implementation of the IPM class with a ROS-Link, where a webcam will be publishing images.
*/
