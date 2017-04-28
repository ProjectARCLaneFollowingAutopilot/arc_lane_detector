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
// Current two lines.
float theta_left;
float rho_left;
float theta_right;
float rho_right;
// Lines.
vector<Vec2f> lines;
// Variable to set initial lane once.
int init_counter = 0;
// Vector to save initialization points.
cv::Point2f init_points[4];
// Save the incoming image.
Mat new_image;

// Functions declarations.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image);
// Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr);
// Function to prompt the user to set four input control points, which are on the two lines.
void setCtrlPts();

int main(int argc, char* argv[])
{
  // Initialize ROS Node.
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle n;

  // Declare callback function to get Webcam image.
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, webcamCallback);

  // Run that shit.
  ros::spin();

  return 0;
}

/*
This program is a playground for Nikhilesh.
* Rosnode
* Subscribe to Webcam image.
* Prompt user to detect two lines (our lane).
* Save the current and previous line parameters of the lines.
* Save a vector "lines", which stores the line parameters of all detected line.
* Iterate through all lines of "lines" to find the two best matches to the previous line.
* Draw line into the image.
*/

// Function definitions.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
  // Create a pointer, where the incoming ros-image gets assigned to.
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);
  // Check for valid image.
  if (!(cv_ptr->image).data)
  {
    std::cout<<"Got bad image from webcam!"<<std::endl;
  }
  // Flip and save the image to global variable.
  cv::flip(cv_ptr->image, new_image, 0);

  // Initialize the image.
  if(init_counter == 0)
  {
    // Prompt user to select two lines.
    setCtrlPts();

    // Avoid anothet initialization when function gets called again.
    init_counter = init_counter + 1;

    // Display image with initialized lines.
    cv::line(new_image, init_points[0], init_points[1], Scalar(0,255,0), 3, 8);
    cv::line(new_image, init_points[2], init_points[3], Scalar(0,255,0), 3, 8);
    cv::imshow("Initlines", new_image);
    cv::waitKey(5000);
  }

  /*
  {
    Robin does Hough Line Transform here.
    Gives a vector which holds all theta-rho-pairs for each detected line.
  }
  */

  std::cout<<"Calibration done. Now in destroyer mode."<<std::endl;
  // Iterate through all lines to find the line which is closest to the previous lines.
  // Initialize counter variables with ridiculously high values.
  float cost_left = 10000.0;
  float cost_right = 10000.0;
  int minimal_cost_left = 1;
  int minimal_cost_right = 1;
  for(int i = 0; i < lines.size(); i++)
  {
    // For each line calculate the relative error in rho (left).
    float rel_error_rho_left_loop = (lines[i][0] - rho_left)/rho_left;
    // For each line calculate the relative error in rho (right).
    float rel_error_rho_right_loop = (lines[i][0] - rho_right)/rho_right;
    // For each line calculate the relative error in theta (left).
    float rel_error_theta_left_loop = (lines[i][1] - theta_left)/theta_left;
    // For each line calculate the relative error in theta (right).
    float rel_error_theta_right_loop = (lines[i][1] - theta_left)/theta_left;
    // For each line calculate a cost function (left).
    float cost_left_loop = sqrt(pow(rel_error_rho_left_loop, 2) + pow(rel_error_theta_left_loop, 2));
    // For each line calculate a cost function (right).
    float cost_right_loop = sqrt(pow(rel_error_rho_right_loop, 2) + pow(rel_error_theta_right_loop, 2));
    // If the cost is lower than the previous, save the line as the correct one (left).
    if(cost_left_loop < cost_left)
    {
      minimal_cost_left = i;
    }
    // If the cost is lower than the previous, save the line as the correct one (right).
    if(cost_right_loop < cost_right)
    {
      minimal_cost_right = i;
    }
  }
  // Save the parameters of the closest lines to global variable.
  rho_left = lines[minimal_cost_left][0];
  theta_left = lines[minimal_cost_left][1];
  rho_right = lines[minimal_cost_right][0];
  theta_right = lines[minimal_cost_right][1];
}

// Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr)
{
  if(event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout<<"CLICK"<<std::endl;
    cv::Point2f *p = (cv::Point2f*)ptr;
    p->x = x;
    p->y = y;
  }
}

// Function to prompt the user to set four input control points, which are on the two lines.
void setCtrlPts()
{
  // Let user select the input points.
  cv::Point2f p;
  cv::namedWindow("Set Control Points", CV_WINDOW_AUTOSIZE);

  // Prompt user to select four points in perspectively distorted input image of the groundplane.
  std::cout<<"Select two lines to Initialize."<<std::endl;
  for(int i = 0; i < 4; i++)
  {
    std::cout<<"Point: "<<i+1<<" out of "<<4<<std::endl;
    std::cout<<"You have now 5 sec to click on your point"<<std::endl;
    cv::imshow("Set Control Points", new_image);
    cv::setMouseCallback("Set Control Points", getClickedPixel, &p);
    cv::waitKey(5000);
    std::cout<<"Saved pixels: "<<std::endl;
    std::cout<<p<<std::endl;
    init_points[i] = p;
    std::cout<<"Input Points saved!"<<std::endl;
  }
  setMouseCallback("Set Control Points",NULL);
}
