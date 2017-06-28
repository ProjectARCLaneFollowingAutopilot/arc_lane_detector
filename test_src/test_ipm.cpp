#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string.h>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "../../arc_lane_tracking_tools/include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"
#include "../../arc_lane_tracking_tools/include/constants.hpp"

using namespace cv;
using namespace std;

// For image->world trafo:
float camera_height = 1.3;
float camera_angle = 90.0;
float focal_length = 628.0;
int counter = 0;
IPM ipm_object;
Mat src;
cv_bridge::CvImagePtr cv_ptr;
int save_counter = 0;


// FUNCTION DECLARATIONS.

// NIKHILESH'S FUNCTIONS.
// Callback function which fetches new images and gets shit done.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image);

int main(int argc, char* argv[])
{
  // Initialize ROS Node.
  ros::init(argc, argv, "ipm_node_test");
  ros::NodeHandle n;

  // Declare callback function to get Webcam image.
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, webcamCallback);

  // Run that shit.
  ros::spin();

  return 0;
}

// FUNCTION DEFINITIONS.
// NIKHILESH'S FUNCTIONS.
// Callback function which fetches new images and gets shit done.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
  // Save incoming image.
  cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);
  // Check for valid image.
  if (!(cv_ptr->image).data)
  {
    std::cout<<"Got bad image from webcam!"<<std::endl;
  }
  // Flip and save the image to global variable.

  cv::flip(cv_ptr->image, src, -1);
  //src = cv_ptr->image;

  ipm_object.IPM::getImage(src);

  if(counter == 0)
  {
    ipm_object.IPM::setParam(camera_height, camera_angle, focal_length, 640, 480);
    counter += 1;
  }

  Mat topview =  ipm_object.IPM::invPerspectiveMapping();
  imshow("Topview", topview);

  if(save_counter == 60)
  {
    imwrite("/home/nikhilesh/Desktop/IPMcomparison/klausenuri1_orig.png", src);
    imwrite("/home/nikhilesh/Desktop/IPMcomparison/klausenuri1_ipm.png", topview);
  }
  waitKey(1);

  save_counter += 1;
}
